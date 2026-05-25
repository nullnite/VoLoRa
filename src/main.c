// VoLoRa — Push-to-Talk voice over LoRa

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

// Radio config
#define RF_FREQ 868123456
#define TX_POWER 14
#define RF_BW BW_125_KHZ
#define RF_SF SF_7
#define RF_CR CR_4_5
#define RF_PREAMBLE 8

// Codec2 2400 bps constants
#define CODEC2_MODE_VAL CODEC2_MODE_2400
#define C2_NSAM 160  // samples per frame (8 kHz × 20 ms)
#define C2_NBYTE 6   // encoded bytes per frame
#define FRAMES_PER_PKT 4

// Packet layout: [1 byte header][FRAMES_PER_PKT × C2_NBYTE payload] = 25 bytes
//   bits [7:4]  4-bit rolling sequence number (0–15)
//   bit  [3]    NEW_SESSION — set on first packet of each PTT press
//   bits [2:0]  reserved
#define PKT_PAYLOAD (FRAMES_PER_PKT * C2_NBYTE)  // 24 bytes
#define PKT_LEN (1 + PKT_PAYLOAD)                // 25 bytes
#define HDR_SEQ_SHIFT 4
#define HDR_SEQ_MASK 0xF0U
#define HDR_NEW_SESSION BIT(3)

// Audio config
#define PDM_SAMPLE_RATE 16000
#define PDM_MIN_CLK_16K 1000000                                  // 16 kHz PCM: 16000×64 = 1.024 MHz -> rounds to 1 MHz
#define PDM_MIN_CLK_8K 512000                                    //  8 kHz PCM:  8000×64 =   512 kHz
#define PDM_BLOCK_SIZE (PDM_SAMPLE_RATE / 50 * sizeof(int16_t))  // 640 B = 320 samples
#define PDM_BLOCK_COUNT 8
#define I2S_SAMPLE_RATE 16000
#define I2S_BLOCK_SAMPLES (C2_NSAM * 2)                           // 320 samples = 20 ms at 16 kHz
#define I2S_BLOCK_SIZE (I2S_BLOCK_SAMPLES * 2 * sizeof(int16_t))  // stereo, 1280 B
#define I2S_BLOCK_COUNT 4

// Hardware nodes
#define AMP_EN_NODE DT_NODELABEL(amp_en)
#define SW3_NODE DT_NODELABEL(button0)   // PTT
#define BTN1_NODE DT_NODELABEL(button1)  // 8 kHz
#define BTN2_NODE DT_NODELABEL(button2)  // 16 kHz
#define LED_DEV_NODE DT_NODELABEL(npm1300_leds)
#define LED_GREEN 1

// Queue depths
#define PCM_Q_DEPTH 8
#define CODED_Q_DEPTH 16

// Queue message types
typedef struct {
    int16_t samples[C2_NSAM];  // 160 samples, 320 bytes
} pcm_frame_t;

typedef struct {
    uint8_t bits[C2_NBYTE];  // 6 bytes
} coded_frame_t;

typedef struct {
    uint8_t data[PKT_LEN];  // 1 header + 24 payload = 25 bytes
    uint16_t len;
    int16_t rssi;
} lora_pkt_t;

// Message queues
K_MSGQ_DEFINE(q1_mic_to_enc, sizeof(pcm_frame_t), PCM_Q_DEPTH, 4);
K_MSGQ_DEFINE(q2_enc_to_tx, sizeof(coded_frame_t), CODED_Q_DEPTH, 4);
K_MSGQ_DEFINE(q3_rx_to_dec, sizeof(lora_pkt_t), 8, 4);
K_MSGQ_DEFINE(q4_dec_to_spk, sizeof(pcm_frame_t), PCM_Q_DEPTH, 4);

// Memory slabs for DMA buffers
K_MEM_SLAB_DEFINE_STATIC(pdm_slab, PDM_BLOCK_SIZE, PDM_BLOCK_COUNT, 4);
K_MEM_SLAB_DEFINE_STATIC(i2s_slab, I2S_BLOCK_SIZE, I2S_BLOCK_COUNT, 4);

// Shared state
static const struct device* dmic_dev;
static const struct device* i2s_dev;
static const struct device* lora_dev;
static const struct device* led_dev;
static struct gpio_dt_spec amp_en;
static struct gpio_dt_spec sw3;
static struct gpio_dt_spec btn1;
static struct gpio_dt_spec btn2;
static struct gpio_callback sw3_cb_data;
static struct gpio_callback btn1_cb_data;
static struct gpio_callback btn2_cb_data;

static struct CODEC2* codec2;

static volatile bool tx_active;
static volatile bool mode_16khz = true;
static volatile bool spk_mode_changed;
static volatile bool mic_mode_changed;

// PTT button press and release
static void ptt_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
    tx_active = (gpio_pin_get_dt(&sw3) != 0);
}

static void btn1_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
    mode_16khz = false;
    spk_mode_changed = true;
    mic_mode_changed = true;
    printk("[AUD] 8 kHz mode\n");
}

static void btn2_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
    mode_16khz = true;
    spk_mode_changed = true;
    mic_mode_changed = true;
    printk("[AUD] 16 kHz mode\n");
}

// Thread 1: Mic (TX path)
//   16 kHz mode: PDM at 16 kHz, 2:1 FIR decimation -> 8 kHz for Codec2
//    8 kHz mode: PDM at  8 kHz, direct copy -> 8 kHz for Codec2
static void mic_thread_fn(void* a, void* b, void* c) {
    struct pcm_stream_cfg stream = {
        .pcm_width = 16,
        .mem_slab = &pdm_slab,
    };
    struct dmic_cfg cfg = {
        .io = {
            .min_pdm_clk_freq = PDM_MIN_CLK_16K,  // updated per mode in MIC_RECONFIGURE
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

    // 8-tap Hamming-windowed sinc, fc = 4 kHz at 16 kHz input.
    // Removes frequencies that would alias after 2:1 decimation.
    static const float dec_fir[8] = {
        -0.00514f,
        -0.02279f,
        0.09637f,
        0.42956f,
        0.42956f,
        0.09637f,
        -0.02279f,
        -0.00514f,
    };
    static int16_t dec_hist[7];  // last 7 input samples carried across blocks

    bool local_16khz = true;

// Apply the current mode_16khz to stream/cfg, then configure and start PDM.
// dmic_configure resets the nRF PDM gain registers, so need to set +20 dB again.
#define MIC_RECONFIGURE()                                                         \
    do {                                                                          \
        local_16khz = mode_16khz;                                                 \
        stream.pcm_rate = local_16khz ? PDM_SAMPLE_RATE : 8000u;                  \
        stream.block_size = local_16khz ? PDM_BLOCK_SIZE                          \
                                        : (uint32_t)(C2_NSAM * sizeof(int16_t));  \
        cfg.io.min_pdm_clk_freq = local_16khz ? PDM_MIN_CLK_16K : PDM_MIN_CLK_8K; \
        memset(dec_hist, 0, sizeof(dec_hist));                                    \
        if (dmic_configure(dmic_dev, &cfg) < 0) {                                 \
            printk("[MIC] configure failed\n");                                   \
        } else {                                                                  \
            NRF_PDM0_S->GAINL = 0x50;                                             \
            NRF_PDM0_S->GAINR = 0x50;                                             \
            dmic_trigger(dmic_dev, DMIC_TRIGGER_START);                           \
        }                                                                         \
        mic_mode_changed = false;                                                 \
        printk("[MIC] %d kHz\n", local_16khz ? 16 : 8);                           \
    } while (0)

    MIC_RECONFIGURE();

    while (true) {
        // Mode switch: stop PDM, reconfigure, restart
        if (mic_mode_changed) {
            dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
            MIC_RECONFIGURE();
        }

        void* buffer;
        uint32_t size;
        int ret = dmic_read(dmic_dev, 0, &buffer, &size, 200);

        if (ret < 0) {
            continue;
        }

        if (!tx_active) {
            // Discard and keep the DMA pipeline draining
            k_mem_slab_free(&pdm_slab, buffer);
            continue;
        }

        const int16_t* in = (const int16_t*)buffer;
        pcm_frame_t frame;

        if (local_16khz) {
            // 2:1 decimation with anti-alias FIR.
            // Output sample n uses input positions 2n, 2n-1, ..., 2n-7.
            int n_in = (int)(size / sizeof(int16_t));  // 320 samples
            for (int n = 0; n < C2_NSAM; n++) {
                float acc = 0.0f;
                for (int k = 0; k < 8; k++) {
                    int idx = 2 * n - k;
                    float s = (idx >= 0) ? (float)in[idx]
                                         : (float)dec_hist[7 + idx];
                    acc += dec_fir[k] * s;
                }
                frame.samples[n] = (int16_t)CLAMP(acc, -32768.0f, 32767.0f);
            }
            for (int i = 0; i < 7; i++) {
                dec_hist[i] = in[n_in - 7 + i];
            }
        } else {
            // 8 kHz: block is exactly C2_NSAM samples, copy directly
            memcpy(frame.samples, in, C2_NSAM * sizeof(int16_t));
        }

        k_mem_slab_free(&pdm_slab, buffer);
        k_msgq_put(&q1_mic_to_enc, &frame, K_NO_WAIT);
    }
}

// Thread 2: Speaker (RX path)
//   Decoded 8 kHz PCM from Q4, upsampled to 16 kHz (or direct at 8 kHz), output via I2S.

// Alloc a slab block, fill it from frame, hand it to i2s_write.
//   mode_16k=true:  2:1 linear interpolation upsample (8→16 kHz, 1280 B)
//   mode_16k=false: direct stereo copy at 8 kHz (640 B, lower half of slab block)
static int spk_write_frame(const pcm_frame_t* frame, bool mode_16k) {
    void* buf;

    if (k_mem_slab_alloc(&i2s_slab, &buf, K_MSEC(100)) != 0) {
        return -ENOMEM;
    }

    int16_t* out = (int16_t*)buf;
    static int16_t prev = 0;
    uint32_t write_size;

    if (mode_16k) {
        // 2:1 linear interpolation upsample (8 kHz -> 16 kHz).
        // Each input sample i produces two output samples:
        //   [2i]   = midpoint between prev and curr  (interpolated)
        //   [2i+1] = curr                            (held)
        for (int i = 0; i < C2_NSAM; i++) {
            int16_t curr = frame->samples[i];
            int16_t interp = (int16_t)(((int32_t)prev + curr + 1) >> 1);
            out[i * 4 + 0] = interp;  // L interpolated
            out[i * 4 + 1] = interp;  // R interpolated
            out[i * 4 + 2] = curr;    // L held
            out[i * 4 + 3] = curr;    // R held
            prev = curr;
        }
        write_size = I2S_BLOCK_SIZE;  // 1280 B
    } else {
        // Direct stereo copy at 8 kHz — no upsampling
        for (int i = 0; i < C2_NSAM; i++) {
            out[i * 2 + 0] = frame->samples[i];  // L
            out[i * 2 + 1] = frame->samples[i];  // R
            prev = frame->samples[i];
        }
        write_size = C2_NSAM * 2 * sizeof(int16_t);  // 640 B
    }

    int ret = i2s_write(i2s_dev, buf, write_size);

    if (ret != 0) {
        k_mem_slab_free(&i2s_slab, buf);
    }

    return ret;
}

static void speaker_thread_fn(void* a, void* b, void* c) {
    static const pcm_frame_t silence;  // zero-initialised

    bool i2s_running = false;
    bool amp_on = false;
    int silent_frames = 0;
    bool local_16khz = true;  // mode active in the current I2S session

    while (true) {
        // Mode switch: stop I2S so it restarts with the new sample rate
        if (i2s_running && spk_mode_changed) {
            printk("[SPK] mode switch — restarting\n");
            i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
            gpio_pin_set_dt(&amp_en, 0);
            led_off(led_dev, LED_GREEN);
            i2s_running = false;
            amp_on = false;
            spk_mode_changed = false;
        }

        pcm_frame_t frame;
        bool have_audio = (k_msgq_get(&q4_dec_to_spk, &frame, K_NO_WAIT) == 0);

        // Not yet started: wait for the first real audio frame
        if (!i2s_running) {
            if (!have_audio) {
                k_msleep(5);
                continue;
            }

            local_16khz = mode_16khz;
            spk_mode_changed = false;

            struct i2s_config i2s_cfg = {
                .word_size = 16,
                .channels = 2,
                .format = I2S_FMT_DATA_FORMAT_I2S,
                .options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
                .frame_clk_freq = local_16khz ? I2S_SAMPLE_RATE : 8000,
                .mem_slab = &i2s_slab,
                .block_size = local_16khz ? I2S_BLOCK_SIZE
                                          : (uint32_t)(C2_NSAM * 2 * sizeof(int16_t)),
                .timeout = 1000,
            };

            printk("[SPK] start %d kHz\n", local_16khz ? 16 : 8);
            gpio_pin_set_dt(&amp_en, 1);
            amp_on = true;
            led_on(led_dev, LED_GREEN);
            i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);

            // Pre-fill 2 slots
            if (spk_write_frame(&frame, local_16khz) != 0) {
                gpio_pin_set_dt(&amp_en, 0);
                amp_on = false;
                led_off(led_dev, LED_GREEN);
                k_msleep(10);
                continue;
            }

            pcm_frame_t f2 = {0};
            k_msgq_get(&q4_dec_to_spk, &f2, K_NO_WAIT);
            spk_write_frame(&f2, local_16khz);

            i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
            i2s_running = true;
            silent_frames = 0;
            continue;
        }

        // I2S running: write audio or silence to keep DMA fed
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
            // Disable amp after ~200 ms of silence
            if (amp_on && silent_frames > 10) {
                gpio_pin_set_dt(&amp_en, 0);
                led_off(led_dev, LED_GREEN);
                amp_on = false;
            }
        }

        int ret = spk_write_frame(to_write, local_16khz);

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

// Thread 3: Codec2 Encode
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

// Thread 4: Codec2 Decode
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
            k_msgq_put(&q4_dec_to_spk, &frame, K_MSEC(50));
        }
    }
}

// Thread 5: LoRa TX
//   Collects FRAMES_PER_PKT encoded frames from Q2, transmits one packet.
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

        // Switch to TX once per PTT press
        if (!in_tx_mode) {
            k_mutex_lock(&radio_mutex, K_FOREVER);
            lora_configure(true);
            k_mutex_unlock(&radio_mutex);
            in_tx_mode = true;
        }

        // Build header
        uint8_t hdr = (tx_seq & 0x0FU) << HDR_SEQ_SHIFT;
        if (!session_started) {
            hdr |= HDR_NEW_SESSION;
            session_started = true;
        }
        tx_seq = (tx_seq + 1) & 0x0FU;
        pkt_buf[0] = hdr;

        // Collect exactly FRAMES_PER_PKT frames, pad with zeros if TX ends early
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

// Thread 6: LoRa RX
//   Blocking receive with a short timeout so PTT can interrupt it.
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

        // Re-check after acquiring, TX may have just started
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

// Thread stacks
K_THREAD_STACK_DEFINE(mic_stack, 2048);
K_THREAD_STACK_DEFINE(speaker_stack, 4096);
K_THREAD_STACK_DEFINE(encode_stack, 16384);
K_THREAD_STACK_DEFINE(decode_stack, 16384);
K_THREAD_STACK_DEFINE(lora_tx_stack, 2048);
K_THREAD_STACK_DEFINE(lora_rx_stack, 2048);

static struct k_thread mic_td, speaker_td, encode_td, decode_td, lora_tx_td, lora_rx_td;

// Entry point
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

    // Device handles
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

    btn1 = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(BTN1_NODE, gpios);
    btn2 = (struct gpio_dt_spec)GPIO_DT_SPEC_GET(BTN2_NODE, gpios);

    // GPIO setup
    gpio_pin_configure_dt(&amp_en, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&sw3, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&sw3, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&sw3_cb_data, ptt_isr, BIT(sw3.pin));
    gpio_add_callback(sw3.port, &sw3_cb_data);

    gpio_pin_configure_dt(&btn1, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&btn1, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&btn1_cb_data, btn1_isr, BIT(btn1.pin));
    gpio_add_callback(btn1.port, &btn1_cb_data);

    gpio_pin_configure_dt(&btn2, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&btn2, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&btn2_cb_data, btn2_isr, BIT(btn2.pin));
    gpio_add_callback(btn2.port, &btn2_cb_data);

    // LoRa gets configured by TX and RX threads

    // Codec2 instance
    codec2 = codec2_create(CODEC2_MODE_VAL);
    if (!codec2) {
        printk("Codec2 init failed\n");
        return -1;
    }
    codec2_set_natural_or_gray(codec2, 0);

    // Start threads
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
