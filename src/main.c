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
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "codec2.h"

/* ── Radio config ─────────────────────────────────────────────────────────── */
#define RF_FREQ 868123456
#define TX_POWER 14
#define RF_BW BW_125_KHZ
#define RF_SF SF_7
#define RF_CR CR_4_5
#define RF_PREAMBLE 8

/* ── Codec2 2400 bps constants ────────────────────────────────────────────── */
#define CODEC2_MODE_VAL CODEC2_MODE_2400
#define C2_NSAM 160 /* samples per frame (8 kHz × 20 ms) */
#define C2_NBYTE 6  /* encoded bytes per frame */
#define FRAMES_PER_PKT 4

/* Packet layout: [1 byte header][FRAMES_PER_PKT × C2_NBYTE payload] = 25 bytes
 *
 * Header byte:
 *   bits [7:4]  4-bit rolling sequence number (0–15)
 *   bit  [3]    NEW_SESSION — set on first packet of each PTT press
 *   bits [2:0]  reserved
 */
#define PKT_PAYLOAD (FRAMES_PER_PKT * C2_NBYTE) /* 24 bytes */
#define PKT_LEN (1 + PKT_PAYLOAD)               /* 25 bytes */
#define HDR_SEQ_SHIFT 4
#define HDR_SEQ_MASK 0xF0U
#define HDR_NEW_SESSION BIT(3)
#define HDR_BENCHMARK BIT(2)      /* packet is part of a benchmark run */
#define HDR_BENCHMARK_LAST BIT(1) /* last packet of the benchmark run */

/* ── Benchmark config ─────────────────────────────────────────────────────── */
/* Uncomment to compile in benchmark mode: PTT sends 100 dummy packets instead
   of voice; the receiver counts and reports how many arrived. */
#define BENCHMARK_MODE
#define BENCHMARK_PKT_COUNT 100

/* ── Audio config ─────────────────────────────────────────────────────────── */
/* Request 8 kHz directly from the DMIC driver; it handles decimation internally
   with a better filter than our 2-tap pair-average. */
#define PDM_SAMPLE_RATE 8000
#define PDM_MIN_CLK 500000
#define PDM_BLOCK_SIZE (PDM_SAMPLE_RATE / 50 * sizeof(int16_t))
#define PDM_BLOCK_COUNT 8
#define I2S_SAMPLE_RATE 8000
#define I2S_BLOCK_SAMPLES C2_NSAM                                /* 160 = 20ms at 8 kHz */
#define I2S_BLOCK_SIZE (I2S_BLOCK_SAMPLES * 2 * sizeof(int16_t)) /* stereo */
#define I2S_BLOCK_COUNT 4

/* ── Hardware nodes ───────────────────────────────────────────────────────── */
#define AMP_EN_NODE DT_NODELABEL(amp_en)
#define SW3_NODE DT_NODELABEL(button0) /* P1.13 */
#define LED_DEV_NODE DT_NODELABEL(npm1300_leds)
#define LED_GREEN 1

/* ── Queue depths ─────────────────────────────────────────────────────────── */
#define PCM_Q_DEPTH 8
#define CODED_Q_DEPTH 16

/* ── Queue message types ──────────────────────────────────────────────────── */
typedef struct {
    int16_t samples[C2_NSAM]; /* 160 samples, 320 bytes */
} pcm_frame_t;

typedef struct {
    uint8_t bits[C2_NBYTE]; /* 6 bytes */
} coded_frame_t;

typedef struct {
    uint8_t data[PKT_LEN]; /* 1 header + 24 payload = 25 bytes */
    uint16_t len;
    int16_t rssi;
} lora_pkt_t;

/* ── Message queues (4 total) ─────────────────────────────────────────────── */
K_MSGQ_DEFINE(q1_mic_to_enc, sizeof(pcm_frame_t), PCM_Q_DEPTH, 4);
K_MSGQ_DEFINE(q2_enc_to_tx, sizeof(coded_frame_t), CODED_Q_DEPTH, 4);
K_MSGQ_DEFINE(q3_rx_to_dec, sizeof(lora_pkt_t), 8, 4);
K_MSGQ_DEFINE(q4_dec_to_spk, sizeof(pcm_frame_t), PCM_Q_DEPTH, 4);

/* ── Memory slabs for DMA ─────────────────────────────────────────────────── */
K_MEM_SLAB_DEFINE_STATIC(pdm_slab, PDM_BLOCK_SIZE, PDM_BLOCK_COUNT, 4);
K_MEM_SLAB_DEFINE_STATIC(i2s_slab, I2S_BLOCK_SIZE, I2S_BLOCK_COUNT, 4);

/* ── Shared state ─────────────────────────────────────────────────────────── */
static const struct device* dmic_dev;
static const struct device* i2s_dev;
static const struct device* lora_dev;
static const struct device* led_dev;
static struct gpio_dt_spec amp_en;
static struct gpio_dt_spec sw3;
static struct gpio_callback sw3_cb_data;

static struct CODEC2* codec2;

static volatile bool tx_active; /* true = PTT held, transmitting */

#ifdef BENCHMARK_MODE
static int bench_rx_count;
static int bench_rx_errors;
static bool bench_rx_active;
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * PTT button interrupt
 * ═══════════════════════════════════════════════════════════════════════════ */
static void ptt_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
    tx_active = (gpio_pin_get_dt(&sw3) != 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 1: Mic (TX path)
 *   Records PDM at 16 kHz, decimates 2:1 to 8 kHz, sends PCM frames on Q1
 * ═══════════════════════════════════════════════════════════════════════════ */
static void mic_thread_fn(void* a, void* b, void* c) {
    struct pcm_stream_cfg stream = {
        .pcm_width = 16,
        .mem_slab = &pdm_slab,
    };
    struct dmic_cfg cfg = {
        .io = {
            .min_pdm_clk_freq = PDM_MIN_CLK,
            .max_pdm_clk_freq = 4000000,
            .min_pdm_clk_dc = 40,
            .max_pdm_clk_dc = 60,
        },
        .streams = &stream,
        .channel = {
            .req_num_streams = 1,
            .req_num_chan = 1,
            .req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT),
        },
    };
    stream.pcm_rate = PDM_SAMPLE_RATE;
    stream.block_size = PDM_BLOCK_SIZE;

    if (dmic_configure(dmic_dev, &cfg) < 0) {
        printk("DMIC configure failed\n");
        return;
    }

    /* +20 dB gain for -26 dBFS mic */
    NRF_PDM0_S->GAINL = 0x50;
    NRF_PDM0_S->GAINR = 0x50;

    bool was_active = false;
    int skip_frames = 0;

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
            skip_frames = 3;
            led_on(led_dev, LED_GREEN);
        }

        void* buffer;
        uint32_t size;
        int64_t t0 = k_uptime_get();
        int ret = dmic_read(dmic_dev, 0, &buffer, &size, 200);
        int64_t dt = k_uptime_get() - t0;

        if (ret < 0) {
            continue;
        }

        if (skip_frames > 0) {
            k_mem_slab_free(&pdm_slab, buffer);
            skip_frames--;
            if (skip_frames == 0) {
                /* Print timing of the first real block */
                printk(
                    "[MIC] block interval=%d ms  size=%u B  "
                    "=> actual_rate~%d Hz\n",
                    (int)dt, size,
                    (int)(1000u * (size / sizeof(int16_t)) / (uint32_t)dt));
            }
            continue;
        }

        pcm_frame_t frame;
        int copy = MIN(C2_NSAM, (int)(size / sizeof(int16_t)));
        memcpy(frame.samples, buffer, copy * sizeof(int16_t));

        k_mem_slab_free(&pdm_slab, buffer);
        k_msgq_put(&q1_mic_to_enc, &frame, K_NO_WAIT);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 2: Speaker (RX path)
 *   Takes decoded 8 kHz PCM from Q4, upsamples to 16 kHz, outputs via I2S
 * ═══════════════════════════════════════════════════════════════════════════ */
/* Alloc a slab block, fill it from frame (upsampled 8→16 kHz, mono→stereo),
   hand it to i2s_write. Returns 0 on success; frees the block on failure. */
static int spk_write_frame(const pcm_frame_t* frame) {
    void* buf;

    if (k_mem_slab_alloc(&i2s_slab, &buf, K_MSEC(100)) != 0) {
        return -ENOMEM;
    }

    int16_t* out = (int16_t*)buf;

    /* I2S runs at 8 kHz — mono→stereo, no upsampling needed */
    for (int i = 0; i < C2_NSAM; i++) {
        out[i * 2] = frame->samples[i];     /* L */
        out[i * 2 + 1] = frame->samples[i]; /* R */
    }

    int ret = i2s_write(i2s_dev, buf, I2S_BLOCK_SIZE);

    if (ret != 0) {
        k_mem_slab_free(&i2s_slab, buf);
    }

    return ret;
}

static void speaker_thread_fn(void* a, void* b, void* c) {
    static const struct i2s_config i2s_cfg = {
        .word_size = 16,
        .channels = 2,
        .format = I2S_FMT_DATA_FORMAT_I2S,
        .options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
        .frame_clk_freq = I2S_SAMPLE_RATE,
        .mem_slab = &i2s_slab,
        .block_size = I2S_BLOCK_SIZE,
        .timeout = 1000,
    };
    static const pcm_frame_t silence; /* zero-initialised */

    bool i2s_running = false;
    bool amp_on = false;
    int silent_frames = 0;

    while (true) {
        pcm_frame_t frame;
        bool have_audio = (k_msgq_get(&q4_dec_to_spk, &frame, K_NO_WAIT) == 0);

        /* ── Not yet started: wait for first real audio frame ── */
        if (!i2s_running) {
            if (!have_audio) {
                k_msleep(5);
                continue;
            }

            printk("[SPK] start\n");
            gpio_pin_set_dt(&amp_en, 1);
            amp_on = true;
            led_on(led_dev, LED_GREEN);
            i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);

            /* Pre-fill 2 slots: nRF I2S double-buffer needs ≥2 before START */
            if (spk_write_frame(&frame) != 0) {
                gpio_pin_set_dt(&amp_en, 0);
                amp_on = false;
                led_off(led_dev, LED_GREEN);
                k_msleep(10);
                continue;
            }

            pcm_frame_t f2 = {0};
            k_msgq_get(&q4_dec_to_spk, &f2, K_NO_WAIT);
            spk_write_frame(&f2);

            i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
            i2s_running = true;
            silent_frames = 0;
            continue;
        }

        /* ── I2S running: write audio or silence to keep DMA fed ── */
        const pcm_frame_t* to_write;

        if (have_audio) {
            to_write = &frame;
            silent_frames = 0;
            if (!amp_on) {
                gpio_pin_set_dt(&amp_en, 1);
                led_on(led_dev, LED_GREEN);
                amp_on = true;
            }
        } else {
            to_write = &silence;
            silent_frames++;
            /* Disable amp after ~200 ms of silence (10 × 20 ms frames) */
            if (amp_on && silent_frames > 10) {
                gpio_pin_set_dt(&amp_en, 0);
                led_off(led_dev, LED_GREEN);
                amp_on = false;
            }
        }

        int ret = spk_write_frame(to_write);

        if (ret != 0) {
            printk("[SPK] err %d — restarting\n", ret);
            i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
            gpio_pin_set_dt(&amp_en, 0);
            led_off(led_dev, LED_GREEN);
            i2s_running = false;
            amp_on = false;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 3: Codec2 Encode (TX path)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void encode_thread_fn(void* a, void* b, void* c) {
    pcm_frame_t frame;
    coded_frame_t coded;

    while (true) {
        if (k_msgq_get(&q1_mic_to_enc, &frame, K_FOREVER) == 0) {
            codec2_encode(codec2, coded.bits, frame.samples);
            k_msgq_put(&q2_enc_to_tx, &coded, K_MSEC(50));
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 4: Codec2 Decode (RX path)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void decode_thread_fn(void* a, void* b, void* c) {
    lora_pkt_t pkt;
    pcm_frame_t frame;
    uint8_t last_seq = 0xFF;

    while (true) {
        if (k_msgq_get(&q3_rx_to_dec, &pkt, K_FOREVER) != 0) {
            continue;
        }
        if (pkt.len < 1 + C2_NBYTE) {
            continue;
        }

        uint8_t hdr = pkt.data[0];
        uint8_t seq = (hdr & HDR_SEQ_MASK) >> HDR_SEQ_SHIFT;
        bool new_session = (hdr & HDR_NEW_SESSION) != 0;

        if (!new_session && last_seq != 0xFF) {
            uint8_t expected = (last_seq + 1) & 0x0FU;
            if (seq != expected) {
                printk("[DEC] lost pkt: expected seq %d got %d\n",
                       expected, seq);
            }
        }
        last_seq = seq;

        int frames = (pkt.len - 1) / C2_NBYTE;

        for (int f = 0; f < frames; f++) {
            codec2_decode(codec2, frame.samples,
                          &pkt.data[1 + f * C2_NBYTE]);
            /* First packet of a new PTT session: warm up decoder state
               but don't play — avoids click/garbage on session start. */
            if (!new_session) {
                k_msgq_put(&q4_dec_to_spk, &frame, K_MSEC(50));
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 5: LoRa TX (TX path)
 *   Collects FRAMES_PER_PKT encoded frames from Q2, transmits packet
 * ═══════════════════════════════════════════════════════════════════════════ */
static K_MUTEX_DEFINE(radio_mutex);

static void lora_configure(bool tx) {
    struct lora_modem_config cfg = {
        .frequency = RF_FREQ,
        .bandwidth = RF_BW,
        .datarate = RF_SF,
        .coding_rate = RF_CR,
        .preamble_len = RF_PREAMBLE,
        .tx_power = TX_POWER,
        .tx = tx,
    };
    lora_config(lora_dev, &cfg);
}

static void lora_tx_thread_fn(void* a, void* b, void* c) {
    uint8_t pkt_buf[PKT_LEN];
    coded_frame_t coded;
    bool in_tx_mode = false;
    bool session_started = false;
    uint8_t tx_seq = 0;

    while (true) {
        if (!tx_active) {
            in_tx_mode = false;
            session_started = false;
            k_msleep(10);
            continue;
        }

        /* Switch to TX once per PTT press */
        if (!in_tx_mode) {
            k_mutex_lock(&radio_mutex, K_FOREVER);
            lora_configure(true);
            k_mutex_unlock(&radio_mutex);
            in_tx_mode = true;
            tx_seq = 0;

#ifdef BENCHMARK_MODE
            printk("[BENCH] TX start — sending %d packets\n", BENCHMARK_PKT_COUNT);
            for (int i = 0; i < BENCHMARK_PKT_COUNT; i++) {
                uint8_t hdr = ((tx_seq & 0x0FU) << HDR_SEQ_SHIFT) | HDR_BENCHMARK;
                if (i == 0)
                    hdr |= HDR_NEW_SESSION;
                if (i == BENCHMARK_PKT_COUNT - 1)
                    hdr |= HDR_BENCHMARK_LAST;
                tx_seq = (tx_seq + 1) & 0x0FU;
                pkt_buf[0] = hdr;
                /* payload = seq value repeated; receiver uses this to detect corruption */
                memset(&pkt_buf[1], hdr >> HDR_SEQ_SHIFT, PKT_PAYLOAD);

                k_mutex_lock(&radio_mutex, K_FOREVER);
                int ret = lora_send(lora_dev, pkt_buf, PKT_LEN);
                k_mutex_unlock(&radio_mutex);
                printk("[BENCH TX] %d/%d%s\n", i + 1, BENCHMARK_PKT_COUNT,
                       ret < 0 ? " ERR" : "");
            }
            printk("[BENCH] TX done\n");
            while (tx_active) {
                k_msleep(10);
            }
            continue;
#endif
        }

        /* Build header */
        uint8_t hdr = (tx_seq & 0x0FU) << HDR_SEQ_SHIFT;
        if (!session_started) {
            hdr |= HDR_NEW_SESSION;
            session_started = true;
        }
        tx_seq = (tx_seq + 1) & 0x0FU;
        pkt_buf[0] = hdr;

        /* Always collect exactly FRAMES_PER_PKT frames; pad zeros if TX ends early */
        for (int f = 0; f < FRAMES_PER_PKT; f++) {
            if (k_msgq_get(&q2_enc_to_tx, &coded, K_MSEC(100)) == 0) {
                memcpy(&pkt_buf[1 + f * C2_NBYTE], coded.bits, C2_NBYTE);
            } else {
                memset(&pkt_buf[1 + f * C2_NBYTE], 0, C2_NBYTE);
            }
        }

        k_mutex_lock(&radio_mutex, K_FOREVER);
        int ret = lora_send(lora_dev, pkt_buf, PKT_LEN);
        k_mutex_unlock(&radio_mutex);

        if (ret < 0) {
            printk("[TX] err %d\n", ret);
        } else {
            printk("[TX] seq=%d%s\n", (hdr >> HDR_SEQ_SHIFT),
                   (hdr & HDR_NEW_SESSION) ? " NEW" : "");
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Thread 6: LoRa RX (RX path)
 *   Blocking receive with short timeout so PTT can interrupt it.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void lora_rx_thread_fn(void* a, void* b, void* c) {
    uint8_t buf[PKT_LEN];
    int16_t rssi;
    int8_t snr;
    bool in_rx_mode = false;

    while (true) {
        if (tx_active) {
            in_rx_mode = false;
            k_msleep(20);
            continue;
        }

        k_mutex_lock(&radio_mutex, K_FOREVER);

        /* Re-check after acquiring: TX may have just started */
        if (tx_active) {
            in_rx_mode = false;
            k_mutex_unlock(&radio_mutex);
            continue;
        }

        if (!in_rx_mode) {
            lora_configure(false);
            in_rx_mode = true;
        }

        int len = lora_recv(lora_dev, buf, sizeof(buf),
                            K_MSEC(100), &rssi, &snr);
        k_mutex_unlock(&radio_mutex);

        if (len > 0) {
#ifdef BENCHMARK_MODE
            uint8_t hdr = buf[0];
            if (hdr & HDR_BENCHMARK) {
                if (hdr & HDR_NEW_SESSION) {
                    if (bench_rx_active) {
                        printk(
                            "[BENCH] RX interrupted — received %d/%d"
                            "  errors %d\n",
                            bench_rx_count, BENCHMARK_PKT_COUNT,
                            bench_rx_errors);
                    }
                    bench_rx_count = 0;
                    bench_rx_errors = 0;
                    bench_rx_active = true;
                    printk("[BENCH] RX session start  RSSI=%d dBm  SNR=%d dB\n",
                           rssi, snr);
                }
                if (bench_rx_active) {
                    bench_rx_count++;

                    /* verify payload: every byte should equal the seq nibble */
                    uint8_t expected = (hdr & HDR_SEQ_MASK) >> HDR_SEQ_SHIFT;
                    for (int b = 1; b < len; b++) {
                        if (buf[b] != expected) {
                            bench_rx_errors++;
                            printk(
                                "[BENCH RX] pkt %d payload error"
                                " (byte %d: got 0x%02x want 0x%02x)\n",
                                bench_rx_count, b, buf[b], expected);
                            break;
                        }
                    }
                }
                if (hdr & HDR_BENCHMARK_LAST) {
                    printk(
                        "[BENCH] RX done — %d/%d packets (%.0f%%)"
                        "  errors %d  RSSI=%d dBm  SNR=%d dB\n",
                        bench_rx_count, BENCHMARK_PKT_COUNT,
                        bench_rx_count * 100.0 / BENCHMARK_PKT_COUNT,
                        bench_rx_errors, rssi, snr);
                    bench_rx_active = false;
                }
            } else
#endif
            {
                printk("[RX] %d B  RSSI=%d dBm  SNR=%d dB\n",
                       len, rssi, snr);
                lora_pkt_t pkt = {0};
                pkt.rssi = rssi;
                memcpy(pkt.data, buf, MIN(len, (int)sizeof(pkt.data)));
                pkt.len = len;
                k_msgq_put(&q3_rx_to_dec, &pkt, K_NO_WAIT);
            }
        }
    }
}

/* ── Thread stacks ────────────────────────────────────────────────────────── */
K_THREAD_STACK_DEFINE(mic_stack, 2048);
K_THREAD_STACK_DEFINE(speaker_stack, 4096);
K_THREAD_STACK_DEFINE(encode_stack, 16384); /* Codec2 needs ~12 KB */
K_THREAD_STACK_DEFINE(decode_stack, 16384);
K_THREAD_STACK_DEFINE(lora_tx_stack, 2048);
K_THREAD_STACK_DEFINE(lora_rx_stack, 2048);

static struct k_thread mic_td, speaker_td, encode_td, decode_td, lora_tx_td, lora_rx_td;

/* ═══════════════════════════════════════════════════════════════════════════
 * Entry point
 * ═══════════════════════════════════════════════════════════════════════════ */
static void wait_for_console(void) {
    const struct device* uart = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

    if (!device_is_ready(uart)) {
        return;
    }
    uint32_t dtr = 0;
    int64_t deadline = k_uptime_get() + 3000;

    while (!dtr && k_uptime_get() < deadline) {
        uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
        k_msleep(100);
    }
}

int main(void) {
    wait_for_console();
    printk("=== VoLoRa Push-to-Talk ===\n");

    /* ── Device handles ── */
    dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
    i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));
    lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
    led_dev = DEVICE_DT_GET(LED_DEV_NODE);
    amp_en = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(AMP_EN_NODE, gpios);
    sw3 = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(SW3_NODE, gpios);

    if (!device_is_ready(dmic_dev)) {
        printk("DMIC not ready\n");
        return -1;
    }
    if (!device_is_ready(i2s_dev)) {
        printk("I2S not ready\n");
        return -1;
    }
    if (!device_is_ready(lora_dev)) {
        printk("LoRa not ready\n");
        return -1;
    }
    if (!device_is_ready(led_dev)) {
        printk("LED not ready\n");
        return -1;
    }

    /* ── GPIO setup ── */
    gpio_pin_configure_dt(&amp_en, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&sw3, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&sw3, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&sw3_cb_data, ptt_isr, BIT(sw3.pin));
    gpio_add_callback(sw3.port, &sw3_cb_data);

    /* LoRa is configured by lora_tx_thread (TX) and lora_rx_thread (RX) */

    /* ── Codec2 instance ── */
    codec2 = codec2_create(CODEC2_MODE_VAL);
    if (!codec2) {
        printk("Codec2 init failed\n");
        return -1;
    }
    codec2_set_natural_or_gray(codec2, 0);

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
