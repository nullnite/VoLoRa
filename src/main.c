/*
 * Codec2 2400 bps voice over LoRa — Zephyr port for nRF5340 + SX1262/LLCC68
 *
 * Role is selected at build time via Kconfig:
 *   CONFIG_LORA_VOICE_TX=y   — encode hts1a test audio and transmit
 *   CONFIG_LORA_VOICE_RX=y   — receive, decode, and output PCM on RTT ch 1
 *
 * PCM output framing (RX):
 *   [0xFF][0xFE][seq][len_lo][len_hi][xor_chk] + 320 bytes raw PCM (s16-le)
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>
#include <SEGGER_RTT.h>
#include <string.h>
#include <stdlib.h>

#include "codec2.h"

#ifdef CONFIG_LORA_VOICE_TX
#include "hts1a.h"
#endif

/* ── Radio config ─────────────────────────────────────────────────────────── */
#define RF_FREQ        868000000
#define TX_POWER       14
#define RF_BW          BW_125_KHZ
#define RF_SF          SF_7
#define RF_CR          CR_4_5
#define RF_PREAMBLE    8

/* ── Codec2 2400 bps constants ────────────────────────────────────────────── */
#define CODEC2_MODE_VAL   CODEC2_MODE_2400
#define FRAMES_PER_PKT    4
#define C2_NSAM           160   /* samples per frame */
#define C2_NBYTE          6     /* encoded bytes per frame */
#define PKT_LEN           (FRAMES_PER_PKT * C2_NBYTE)  /* 24 bytes */

/* ── RTT channel 1 for binary PCM output ─────────────────────────────────── */
#define RTT_PCM_CH  1
static uint8_t rtt_pcm_buf[2048];

static void rtt_write_pcm(const short *frame, uint8_t seq)
{
	uint8_t header[6];
	const uint8_t *raw = (const uint8_t *)frame;
	uint8_t chk = 0;

	for (int i = 0; i < C2_NSAM * 2; i++) {
		chk ^= raw[i];
	}
	header[0] = 0xFF;
	header[1] = 0xFE;
	header[2] = seq;
	header[3] = (uint8_t)((C2_NSAM * 2) & 0xFF);
	header[4] = (uint8_t)(((C2_NSAM * 2) >> 8) & 0xFF);
	header[5] = chk;

	SEGGER_RTT_Write(RTT_PCM_CH, header, sizeof(header));
	SEGGER_RTT_Write(RTT_PCM_CH, raw, C2_NSAM * 2);
}

/* ── Shared state ─────────────────────────────────────────────────────────── */
static const struct device *lora_dev;
static struct CODEC2 *codec2;

/* ═══════════════════════════════════════════════════════════════════════════
 * TX path
 * ═══════════════════════════════════════════════════════════════════════════ */
#ifdef CONFIG_LORA_VOICE_TX

static K_SEM_DEFINE(enc_start_sem, 0, 1);
static K_SEM_DEFINE(enc_done_sem,  0, 1);

static short   pcm_buf[C2_NSAM];
static uint8_t enc_buf[C2_NBYTE];
static uint8_t pkt_buf[PKT_LEN];
static int     sample_pos;

/* Runs codec2_encode in its own thread to overlap with packet transmission. */
static void codec2_encode_fn(void *a, void *b, void *c)
{
	while (true) {
		k_sem_take(&enc_start_sem, K_FOREVER);
		codec2_encode(codec2, enc_buf, pcm_buf);
		k_sem_give(&enc_done_sem);
	}
}

static void encode_next_frame(void)
{
	for (int i = 0; i < C2_NSAM; i++) {
		pcm_buf[i] = hts1a_samples[sample_pos++];
		if (sample_pos >= hts1a_num_samples) {
			sample_pos = 0;
		}
	}
	k_sem_give(&enc_start_sem);
	k_sem_take(&enc_done_sem, K_FOREVER);
}

static void fill_packet(uint8_t *pkt)
{
	for (int f = 0; f < FRAMES_PER_PKT; f++) {
		encode_next_frame();
		memcpy(pkt + f * C2_NBYTE, enc_buf, C2_NBYTE);
	}
}

static void tx_fn(void *a, void *b, void *c)
{
	while (true) {
		fill_packet(pkt_buf);
		int ret = lora_send(lora_dev, pkt_buf, PKT_LEN);

		if (ret < 0) {
			printk("LoRa TX err: %d\n", ret);
		}
	}
}

K_THREAD_STACK_DEFINE(enc_stack, 16384);
K_THREAD_STACK_DEFINE(tx_stack,   2048);
static struct k_thread enc_td, tx_td;

#endif /* CONFIG_LORA_VOICE_TX */

/* ═══════════════════════════════════════════════════════════════════════════
 * RX path
 * ═══════════════════════════════════════════════════════════════════════════ */
#ifdef CONFIG_LORA_VOICE_RX

static K_SEM_DEFINE(dec_start_sem, 0, 1);
static K_SEM_DEFINE(dec_done_sem,  0, 1);

typedef struct {
	uint8_t  data[PKT_LEN];
	uint16_t len;
	int16_t  rssi;
} RawPacket;

K_MSGQ_DEFINE(raw_pkt_q, sizeof(RawPacket), 32, 4);
K_MSGQ_DEFINE(pcm_out_q, C2_NSAM * sizeof(short), 20, 4);

static short    pcm_buf[C2_NSAM];
static uint8_t *dec_frame_ptr;

static void rx_callback(const struct device *dev, uint8_t *payload,
			uint16_t size, int16_t rssi, int8_t snr,
			void *user_data)
{
	RawPacket pkt = {0};

	pkt.rssi = rssi;
	if (size <= sizeof(pkt.data)) {
		memcpy(pkt.data, payload, size);
		pkt.len = size;
	}
	k_msgq_put(&raw_pkt_q, &pkt, K_NO_WAIT);
}

static void codec2_decode_fn(void *a, void *b, void *c)
{
	while (true) {
		k_sem_take(&dec_start_sem, K_FOREVER);
		codec2_decode(codec2, pcm_buf, dec_frame_ptr);
		k_sem_give(&dec_done_sem);
	}
}

static void process_packet(const uint8_t *data, int len)
{
	int frames = len / C2_NBYTE;

	for (int f = 0; f < frames; f++) {
		/* dec_frame_ptr handoff is safe: only this thread drives the
		 * dec_start/dec_done semaphore pair */
		dec_frame_ptr = (uint8_t *)(data + f * C2_NBYTE);
		k_sem_give(&dec_start_sem);
		k_sem_take(&dec_done_sem, K_FOREVER);
		k_msgq_put(&pcm_out_q, pcm_buf, K_NO_WAIT);
	}
}

static void decode_fn(void *a, void *b, void *c)
{
	RawPacket pkt;

	while (true) {
		k_msgq_get(&raw_pkt_q, &pkt, K_FOREVER);
		if (pkt.len > 0) {
			process_packet(pkt.data, pkt.len);
		}
	}
}

static void serial_out_fn(void *a, void *b, void *c)
{
	short frame[C2_NSAM];
	uint8_t seq = 0;

	while (true) {
		k_msgq_get(&pcm_out_q, frame, K_FOREVER);
		rtt_write_pcm(frame, seq++);
	}
}

K_THREAD_STACK_DEFINE(dec_codec_stack, 16384);
K_THREAD_STACK_DEFINE(decode_stack,    2048);
K_THREAD_STACK_DEFINE(serout_stack,    1024);
static struct k_thread dec_codec_td, decode_td, serout_td;

#endif /* CONFIG_LORA_VOICE_RX */

/* ═══════════════════════════════════════════════════════════════════════════
 * Entry point
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
	printk("=== Codec2 LoRa %s ===\n",
	       IS_ENABLED(CONFIG_LORA_VOICE_TX) ? "TX" : "RX");

	lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
	if (!device_is_ready(lora_dev)) {
		printk("LoRa not ready\n");
		return -1;
	}

	struct lora_modem_config cfg = {
		.frequency    = RF_FREQ,
		.bandwidth    = RF_BW,
		.datarate     = RF_SF,
		.coding_rate  = RF_CR,
		.preamble_len = RF_PREAMBLE,
		.tx_power     = TX_POWER,
		.tx           = IS_ENABLED(CONFIG_LORA_VOICE_TX),
	};
	if (lora_config(lora_dev, &cfg) < 0) {
		printk("LoRa config failed\n");
		return -1;
	}

	codec2 = codec2_create(CODEC2_MODE_VAL);
	if (!codec2) {
		printk("Codec2 init failed\n");
		return -1;
	}
	codec2_set_natural_or_gray(codec2, 0);

	SEGGER_RTT_ConfigUpBuffer(RTT_PCM_CH, "PCM",
				  rtt_pcm_buf, sizeof(rtt_pcm_buf),
				  SEGGER_RTT_MODE_NO_BLOCK_SKIP);

#ifdef CONFIG_LORA_VOICE_TX
	k_thread_create(&enc_td, enc_stack, K_THREAD_STACK_SIZEOF(enc_stack),
			codec2_encode_fn, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	k_thread_name_set(&enc_td, "c2enc");

	k_thread_create(&tx_td, tx_stack, K_THREAD_STACK_SIZEOF(tx_stack),
			tx_fn, NULL, NULL, NULL, 3, 0, K_NO_WAIT);
	k_thread_name_set(&tx_td, "lora_tx");

	printk("TX started\n");
#endif

#ifdef CONFIG_LORA_VOICE_RX
	k_thread_create(&dec_codec_td, dec_codec_stack,
			K_THREAD_STACK_SIZEOF(dec_codec_stack),
			codec2_decode_fn, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	k_thread_name_set(&dec_codec_td, "c2dec");

	k_thread_create(&decode_td, decode_stack,
			K_THREAD_STACK_SIZEOF(decode_stack),
			decode_fn, NULL, NULL, NULL, 4, 0, K_NO_WAIT);
	k_thread_name_set(&decode_td, "decode");

	k_thread_create(&serout_td, serout_stack,
			K_THREAD_STACK_SIZEOF(serout_stack),
			serial_out_fn, NULL, NULL, NULL, 3, 0, K_NO_WAIT);
	k_thread_name_set(&serout_td, "serout");

	lora_recv_async(lora_dev, rx_callback, NULL);
	printk("RX started\n");
#endif

	return 0;
}
