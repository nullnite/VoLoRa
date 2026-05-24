/*
 * VoLoRa — Push-to-Talk voice over LoRa
 *
 * Architecture: 6 threads in 3 pairs + 4 message queues
 *
 * TX path (PTT held):
 *   [Mic Thread] --Q1(PCM)--> [Encode Thread] --Q2(coded)--> [LoRa TX Thread]
 *
 * RX path (PTT released):
 *   [LoRa RX Thread] --Q3(coded)--> [Decode Thread] --Q4(PCM)--> [Speaker Thread]
 *
 * Interrupts:
 *   - PTT button (SW3, P1.13): starts/stops TX path
 *   - LoRa RX async callback: feeds Q3
 */

#include <hal/nrf_pdm.h>
#include <string.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>

#include "codec2.h"

/* ── Radio config ─────────────────────────────────────────────────────────── */
#define RF_FREQ        868000000
#define TX_POWER       14
#define RF_BW          BW_125_KHZ
#define RF_SF          SF_7
#define RF_CR          CR_4_5
#define RF_PREAMBLE    8

/* ── Codec2 2400 bps constants ────────────────────────────────────────────── */
#define CODEC2_MODE_VAL   CODEC2_MODE_2400
#define C2_NSAM           160   /* samples per frame (8 kHz × 20 ms) */
#define C2_NBYTE          6     /* encoded bytes per frame */
#define FRAMES_PER_PKT    4
#define PKT_LEN           (FRAMES_PER_PKT * C2_NBYTE)  /* 24 bytes */

/* ── Audio config ─────────────────────────────────────────────────────────── */
#define PDM_SAMPLE_RATE   16000
#define PDM_BLOCK_SIZE    (PDM_SAMPLE_RATE / 50 * sizeof(int16_t))  /* 20ms=320 samp */
#define PDM_BLOCK_COUNT   8
#define I2S_SAMPLE_RATE   16000
#define I2S_BLOCK_SAMPLES 320   /* 20ms at 16 kHz */
#define I2S_BLOCK_SIZE    (I2S_BLOCK_SAMPLES * 2 * sizeof(int16_t)) /* stereo */
#define I2S_BLOCK_COUNT   4

/* ── Hardware nodes ───────────────────────────────────────────────────────── */
#define AMP_EN_NODE    DT_NODELABEL(amp_en)
#define SW3_NODE       DT_NODELABEL(button0)   /* P1.13 */
#define LED_DEV_NODE   DT_NODELABEL(npm1300_leds)
#define LED_GREEN      1

/* ── Queue depths ─────────────────────────────────────────────────────────── */
#define PCM_Q_DEPTH     8
#define CODED_Q_DEPTH   16

/* ── Queue message types ──────────────────────────────────────────────────── */
typedef struct {
	int16_t samples[C2_NSAM];   /* 160 samples, 320 bytes */
} pcm_frame_t;

typedef struct {
	uint8_t bits[C2_NBYTE];     /* 6 bytes */
} coded_frame_t;

typedef struct {
	uint8_t data[PKT_LEN];      /* 24 bytes */
	uint16_t len;
	int16_t rssi;
} lora_pkt_t;

/* ── Message queues (4 total) ─────────────────────────────────────────────── */
K_MSGQ_DEFINE(q1_mic_to_enc, sizeof(pcm_frame_t),  PCM_Q_DEPTH,   4);
K_MSGQ_DEFINE(q2_enc_to_tx,  sizeof(coded_frame_t), CODED_Q_DEPTH, 4);
K_MSGQ_DEFINE(q3_rx_to_dec,  sizeof(lora_pkt_t),   8,             4);
K_MSGQ_DEFINE(q4_dec_to_spk, sizeof(pcm_frame_t),  PCM_Q_DEPTH,   4);

/* ── Memory slabs for DMA ─────────────────────────────────────────────────── */
K_MEM_SLAB_DEFINE_STATIC(pdm_slab, PDM_BLOCK_SIZE,  PDM_BLOCK_COUNT, 4);
K_MEM_SLAB_DEFINE_STATIC(i2s_slab, I2S_BLOCK_SIZE,  I2S_BLOCK_COUNT, 4);

/* ── Shared state ─────────────────────────────────────────────────────────── */
static const struct device *dmic_dev;
static const struct device *i2s_dev;
static const struct device *lora_dev;
static const struct device *led_dev;
static struct gpio_dt_spec amp_en;
static struct gpio_dt_spec sw3;
static struct gpio_callback sw3_cb_data;

static struct CODEC2 *codec2_enc;
static struct CODEC2 *codec2_dec;

static volatile bool tx_active;   /* true = PTT held, transmitting */

/* ═══════════════════════════════════════════════════════════════════════════
 * PTT button interrupt
 * ═══════════════════════════════════════════════════════════════════════════ */
static void ptt_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	tx_active = (gpio_pin_get_dt(&sw3) != 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 1: Mic (TX path)
 *   Records PDM at 16 kHz, decimates 2:1 to 8 kHz, sends PCM frames on Q1
 * ═══════════════════════════════════════════════════════════════════════════ */
static void mic_thread_fn(void *a, void *b, void *c)
{
	struct pcm_stream_cfg stream = {
		.pcm_width = 16,
		.mem_slab  = &pdm_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 1100000,
			.max_pdm_clk_freq = 4000000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams  = 1,
			.req_num_chan     = 1,
			.req_chan_map_lo  = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT),
		},
	};
	stream.pcm_rate  = PDM_SAMPLE_RATE;
	stream.block_size = PDM_BLOCK_SIZE;

	if (dmic_configure(dmic_dev, &cfg) < 0) {
		printk("DMIC configure failed\n");
		return;
	}

	/* +20 dB gain for -26 dBFS mic */
	NRF_PDM0_S->GAINL = 0x50;
	NRF_PDM0_S->GAINR = 0x50;

	bool was_active = false;

	while (true) {
		if (!tx_active) {
			if (was_active) {
				dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
				was_active = false;
			}
			k_msleep(10);
			continue;
		}

		if (!was_active) {
			dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
			was_active = true;
			led_on(led_dev, LED_GREEN);
		}

		void *buffer;
		uint32_t size;
		int ret = dmic_read(dmic_dev, 0, &buffer, &size, 100);
		if (ret < 0) {
			continue;
		}

		/* Decimate 16 kHz → 8 kHz: average pairs */
		int16_t *raw = (int16_t *)buffer;
		pcm_frame_t frame;
		int raw_samples = size / sizeof(int16_t);

		for (int i = 0; i < C2_NSAM && (i * 2 + 1) < raw_samples; i++) {
			frame.samples[i] = (raw[i * 2] + raw[i * 2 + 1]) / 2;
		}

		k_mem_slab_free(&pdm_slab, buffer);
		k_msgq_put(&q1_mic_to_enc, &frame, K_NO_WAIT);
	}
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 2: Speaker (RX path)
 *   Takes decoded 8 kHz PCM from Q4, upsamples to 16 kHz, outputs via I2S
 * ═══════════════════════════════════════════════════════════════════════════ */
static void speaker_thread_fn(void *a, void *b, void *c)
{
	struct i2s_config i2s_cfg = {
		.word_size      = 16,
		.channels       = 2,
		.format         = I2S_FMT_DATA_FORMAT_I2S,
		.options        = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = I2S_SAMPLE_RATE,
		.mem_slab       = &i2s_slab,
		.block_size     = I2S_BLOCK_SIZE,
		.timeout        = 1000,
	};

	bool playing = false;

	while (true) {
		pcm_frame_t frame;

		if (k_msgq_get(&q4_dec_to_spk, &frame, K_MSEC(100)) != 0) {
			if (playing) {
				i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
				gpio_pin_set_dt(&amp_en, 0);
				led_off(led_dev, LED_GREEN);
				playing = false;
			}
			continue;
		}

		if (!playing) {
			gpio_pin_set_dt(&amp_en, 1);
			led_on(led_dev, LED_GREEN);
			i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
			playing = false;  /* will start after prefill */
		}

		/* Upsample 8 kHz → 16 kHz (duplicate) + mono→stereo */
		void *buf;
		if (k_mem_slab_alloc(&i2s_slab, &buf, K_MSEC(50)) != 0) {
			continue;
		}
		int16_t *out = (int16_t *)buf;

		for (int i = 0; i < C2_NSAM; i++) {
			int16_t val = frame.samples[i];
			out[i * 4]     = val;  /* L */
			out[i * 4 + 1] = val;  /* R */
			out[i * 4 + 2] = val;  /* L (duplicate) */
			out[i * 4 + 3] = val;  /* R (duplicate) */
		}

		int ret = i2s_write(i2s_dev, buf, I2S_BLOCK_SIZE);
		if (ret == -EIO) {
			/* Stream not started yet or error — start it */
			i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
			i2s_write(i2s_dev, buf, I2S_BLOCK_SIZE);
			i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
			playing = true;
		} else if (!playing) {
			i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
			playing = true;
		}
	}
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 3: Codec2 Encode (TX path)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void encode_thread_fn(void *a, void *b, void *c)
{
	pcm_frame_t frame;
	coded_frame_t coded;

	while (true) {
		if (k_msgq_get(&q1_mic_to_enc, &frame, K_FOREVER) == 0) {
			codec2_encode(codec2_enc, coded.bits, frame.samples);
			k_msgq_put(&q2_enc_to_tx, &coded, K_MSEC(50));
		}
	}
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 4: Codec2 Decode (RX path)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void decode_thread_fn(void *a, void *b, void *c)
{
	lora_pkt_t pkt;
	pcm_frame_t frame;

	while (true) {
		if (k_msgq_get(&q3_rx_to_dec, &pkt, K_FOREVER) == 0) {
			int frames = pkt.len / C2_NBYTE;

			for (int f = 0; f < frames; f++) {
				codec2_decode(codec2_dec, frame.samples,
					      &pkt.data[f * C2_NBYTE]);
				k_msgq_put(&q4_dec_to_spk, &frame, K_MSEC(50));
			}
		}
	}
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 5: LoRa TX (TX path)
 *   Collects FRAMES_PER_PKT encoded frames from Q2, transmits packet
 * ═══════════════════════════════════════════════════════════════════════════ */
static void lora_tx_thread_fn(void *a, void *b, void *c)
{
	uint8_t pkt_buf[PKT_LEN];
	coded_frame_t coded;

	while (true) {
		if (!tx_active) {
			k_msleep(10);
			continue;
		}

		int collected = 0;
		for (int f = 0; f < FRAMES_PER_PKT; f++) {
			if (k_msgq_get(&q2_enc_to_tx, &coded, K_MSEC(100)) == 0) {
				memcpy(&pkt_buf[f * C2_NBYTE], coded.bits, C2_NBYTE);
				collected++;
			} else {
				break;
			}
		}

		if (collected > 0) {
			int ret = lora_send(lora_dev, pkt_buf, collected * C2_NBYTE);
			if (ret < 0) {
				printk("LoRa TX err: %d\n", ret);
			}
		}
	}
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 6: LoRa RX (RX path)
 *   Async callback feeds Q3; thread manages RX enable/disable
 * ═══════════════════════════════════════════════════════════════════════════ */
static void lora_rx_callback(const struct device *dev, uint8_t *payload,
			     uint16_t size, int16_t rssi, int8_t snr,
			     void *user_data)
{
	lora_pkt_t pkt = {0};
	pkt.rssi = rssi;

	if (size <= sizeof(pkt.data)) {
		memcpy(pkt.data, payload, size);
		pkt.len = size;
	}
	k_msgq_put(&q3_rx_to_dec, &pkt, K_NO_WAIT);
}

static void lora_rx_thread_fn(void *a, void *b, void *c)
{
	bool rx_enabled = false;

	while (true) {
		if (tx_active) {
			if (rx_enabled) {
				rx_enabled = false;
			}
			k_msleep(20);
		} else {
			if (!rx_enabled) {
				lora_recv_async(lora_dev, lora_rx_callback, NULL);
				rx_enabled = true;
			}
			k_msleep(50);
		}
	}
}

/* ── Thread stacks ────────────────────────────────────────────────────────── */
K_THREAD_STACK_DEFINE(mic_stack,     2048);
K_THREAD_STACK_DEFINE(speaker_stack, 2048);
K_THREAD_STACK_DEFINE(encode_stack,  16384);  /* Codec2 needs ~12 KB */
K_THREAD_STACK_DEFINE(decode_stack,  16384);
K_THREAD_STACK_DEFINE(lora_tx_stack, 2048);
K_THREAD_STACK_DEFINE(lora_rx_stack, 2048);

static struct k_thread mic_td, speaker_td, encode_td, decode_td, lora_tx_td, lora_rx_td;

/* ═══════════════════════════════════════════════════════════════════════════
 * Entry point
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
	printk("=== VoLoRa Push-to-Talk ===\n");

	/* ── Device handles ── */
	dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	i2s_dev  = DEVICE_DT_GET(DT_NODELABEL(i2s0));
	lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
	led_dev  = DEVICE_DT_GET(LED_DEV_NODE);
	amp_en   = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(AMP_EN_NODE, gpios);
	sw3      = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(SW3_NODE, gpios);

	if (!device_is_ready(dmic_dev)) { printk("DMIC not ready\n");  return -1; }
	if (!device_is_ready(i2s_dev))  { printk("I2S not ready\n");   return -1; }
	if (!device_is_ready(lora_dev)) { printk("LoRa not ready\n");  return -1; }
	if (!device_is_ready(led_dev))  { printk("LED not ready\n");   return -1; }

	/* ── GPIO setup ── */
	gpio_pin_configure_dt(&amp_en, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&sw3, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&sw3, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&sw3_cb_data, ptt_isr, BIT(sw3.pin));
	gpio_add_callback(sw3.port, &sw3_cb_data);

	/* ── LoRa config ── */
	struct lora_modem_config lora_cfg = {
		.frequency    = RF_FREQ,
		.bandwidth    = RF_BW,
		.datarate     = RF_SF,
		.coding_rate  = RF_CR,
		.preamble_len = RF_PREAMBLE,
		.tx_power     = TX_POWER,
		.tx           = false,  /* start in RX mode */
	};
	if (lora_config(lora_dev, &lora_cfg) < 0) {
		printk("LoRa config failed\n");
		return -1;
	}

	/* ── Codec2 instances (separate for encode/decode threads) ── */
	codec2_enc = codec2_create(CODEC2_MODE_VAL);
	codec2_dec = codec2_create(CODEC2_MODE_VAL);
	if (!codec2_enc || !codec2_dec) {
		printk("Codec2 init failed\n");
		return -1;
	}
	codec2_set_natural_or_gray(codec2_enc, 0);
	codec2_set_natural_or_gray(codec2_dec, 0);

	/* ── Launch threads ── */
	k_thread_create(&mic_td, mic_stack, K_THREAD_STACK_SIZEOF(mic_stack),
			mic_thread_fn, NULL, NULL, NULL, 4, 0, K_NO_WAIT);
	k_thread_name_set(&mic_td, "mic");

	k_thread_create(&speaker_td, speaker_stack, K_THREAD_STACK_SIZEOF(speaker_stack),
			speaker_thread_fn, NULL, NULL, NULL, 4, 0, K_NO_WAIT);
	k_thread_name_set(&speaker_td, "speaker");

	k_thread_create(&encode_td, encode_stack, K_THREAD_STACK_SIZEOF(encode_stack),
			encode_thread_fn, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	k_thread_name_set(&encode_td, "c2enc");

	k_thread_create(&decode_td, decode_stack, K_THREAD_STACK_SIZEOF(decode_stack),
			decode_thread_fn, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	k_thread_name_set(&decode_td, "c2dec");

	k_thread_create(&lora_tx_td, lora_tx_stack, K_THREAD_STACK_SIZEOF(lora_tx_stack),
			lora_tx_thread_fn, NULL, NULL, NULL, 3, 0, K_NO_WAIT);
	k_thread_name_set(&lora_tx_td, "lora_tx");

	k_thread_create(&lora_rx_td, lora_rx_stack, K_THREAD_STACK_SIZEOF(lora_rx_stack),
			lora_rx_thread_fn, NULL, NULL, NULL, 3, 0, K_NO_WAIT);
	k_thread_name_set(&lora_rx_td, "lora_rx");

	printk("Ready — hold SW3 to talk\n");

	return 0;
}
