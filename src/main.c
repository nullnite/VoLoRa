#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <hal/nrf_pdm.h>
#include <string.h>

#define SAMPLE_RATE	16000
#define SAMPLE_BIT_WIDTH 16
#define MAX_RECORD_SECONDS 10
#define MAX_SAMPLES	(SAMPLE_RATE * MAX_RECORD_SECONDS)
#define MAX_BYTES	(MAX_SAMPLES * sizeof(int16_t))

/* PDM recording: mono, 20ms blocks */
#define PDM_BLOCK_SIZE	(SAMPLE_RATE / 50 * sizeof(int16_t))
#define PDM_BLOCK_COUNT	8

/* I2S playback: stereo, 10ms blocks */
#define I2S_SAMPLES_PER_BLOCK (SAMPLE_RATE / 100)
#define I2S_BLOCK_SIZE	(I2S_SAMPLES_PER_BLOCK * 2 * sizeof(int16_t))
#define I2S_BLOCK_COUNT	4

#define AMP_EN_NODE DT_NODELABEL(amp_en)

/* SW3 = button0 (P1.13, active low) */
#define SW3_NODE DT_NODELABEL(button0)

/* nPM1300 LED1 (host-controlled green LED) */
#define LED_DEV_NODE DT_NODELABEL(npm1300_leds)
#define LED_GREEN 1

/* LED blink toggle every N PDM blocks (N*20ms) */
#define LED_TOGGLE_BLOCKS 5  /* 100ms toggle = fast blink */

K_MEM_SLAB_DEFINE_STATIC(pdm_slab, PDM_BLOCK_SIZE, PDM_BLOCK_COUNT, 4);
K_MEM_SLAB_DEFINE_STATIC(i2s_slab, I2S_BLOCK_SIZE, I2S_BLOCK_COUNT, 4);

/* Audio buffer in RAM */
static int16_t audio_buf[MAX_SAMPLES];

int main(void)
{
	const struct device *dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	const struct device *i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));
	const struct device *led_dev = DEVICE_DT_GET(LED_DEV_NODE);
	const struct gpio_dt_spec amp_en = GPIO_DT_SPEC_GET(AMP_EN_NODE, gpios);
	const struct gpio_dt_spec sw3 = GPIO_DT_SPEC_GET(SW3_NODE, gpios);
	int ret;

	printk("VoLoRa Push-to-Talk\n");
	printk("Hold SW3 to record (max %d s), release to play back\n",
	       MAX_RECORD_SECONDS);

	if (!device_is_ready(dmic_dev)) {
		printk("DMIC device not ready\n");
		return 0;
	}
	if (!device_is_ready(i2s_dev)) {
		printk("I2S device not ready\n");
		return 0;
	}
	if (!gpio_is_ready_dt(&amp_en)) {
		printk("AMP GPIO not ready\n");
		return 0;
	}
	if (!device_is_ready(led_dev)) {
		printk("LED device not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&sw3, GPIO_INPUT);
	gpio_pin_configure_dt(&amp_en, GPIO_OUTPUT_INACTIVE);

	/* --- PDM MIC SETUP --- */
	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab = &pdm_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 1100000,
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
	stream.pcm_rate = SAMPLE_RATE;
	stream.block_size = PDM_BLOCK_SIZE;

	ret = dmic_configure(dmic_dev, &cfg);
	if (ret < 0) {
		printk("DMIC configure failed: %d\n", ret);
		return 0;
	}

	/* +20 dB gain */
	NRF_PDM0_S->GAINL = 0x50;
	NRF_PDM0_S->GAINR = 0x50;

	/* I2S config (reused each playback) */
	struct i2s_config i2s_cfg = {
		.word_size = SAMPLE_BIT_WIDTH,
		.channels = 2,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = SAMPLE_RATE,
		.mem_slab = &i2s_slab,
		.block_size = I2S_BLOCK_SIZE,
		.timeout = 1000,
	};

	printk("Ready. Press SW3 to record.\n");

	while (1) {
		/* Wait for SW3 press (active low) */
		while (gpio_pin_get_dt(&sw3) == 0) {
			k_msleep(10);
		}

		/* Debounce */
		k_msleep(20);
		if (gpio_pin_get_dt(&sw3) == 0) {
			continue;
		}

		/* --- RECORD while button held --- */
		printk("Recording...\n");
		ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
		if (ret < 0) {
			printk("DMIC start failed: %d\n", ret);
			continue;
		}

		size_t offset = 0;
		int max_blocks = MAX_BYTES / PDM_BLOCK_SIZE;
		bool led_state = false;

		for (int i = 0; i < max_blocks; i++) {
			/* Toggle LED every N blocks */
			if (i % LED_TOGGLE_BLOCKS == 0) {
				led_state = !led_state;
				if (led_state) {
					led_on(led_dev, LED_GREEN);
				} else {
					led_off(led_dev, LED_GREEN);
				}
			}

			/* Check if button released */
			if (gpio_pin_get_dt(&sw3) == 0) {
				break;
			}

			void *buffer;
			uint32_t size;

			ret = dmic_read(dmic_dev, 0, &buffer, &size, 1000);
			if (ret < 0) {
				break;
			}

			size_t copy_len = MIN(size, MAX_BYTES - offset);
			memcpy((uint8_t *)audio_buf + offset, buffer, copy_len);
			offset += copy_len;
			k_mem_slab_free(&pdm_slab, buffer);
		}

		dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
		led_off(led_dev, LED_GREEN);

		int16_t min_val = 0, max_val = 0;
		int total_rec_samples = offset / sizeof(int16_t);
		for (int i = 0; i < total_rec_samples; i++) {
			if (audio_buf[i] < min_val) min_val = audio_buf[i];
			if (audio_buf[i] > max_val) max_val = audio_buf[i];
		}
		printk("Recorded %u bytes (%u ms), min=%d max=%d\n",
		       (unsigned int)offset,
		       (unsigned int)(total_rec_samples * 1000 / SAMPLE_RATE),
		       min_val, max_val);

		if (offset == 0) {
			continue;
		}

		/* --- PLAYBACK --- */
		printk("Playing back...\n");
		led_on(led_dev, LED_GREEN);
		gpio_pin_set_dt(&amp_en, 1);

		ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
		if (ret < 0) {
			printk("I2S configure failed: %d\n", ret);
			led_off(led_dev, LED_GREEN);
			gpio_pin_set_dt(&amp_en, 0);
			continue;
		}

		int playback_blocks = (total_rec_samples + I2S_SAMPLES_PER_BLOCK - 1) /
				      I2S_SAMPLES_PER_BLOCK;
		int sample_idx = 0;

		/* Pre-fill TX queue */
		int prefill = MIN(I2S_BLOCK_COUNT - 1, playback_blocks);
		for (int i = 0; i < prefill; i++) {
			void *buf;
			k_mem_slab_alloc(&i2s_slab, &buf, K_FOREVER);
			int16_t *out = (int16_t *)buf;

			for (int s = 0; s < I2S_SAMPLES_PER_BLOCK; s++) {
				int16_t val = (sample_idx < total_rec_samples) ?
					      audio_buf[sample_idx++] : 0;
				out[s * 2] = val;
				out[s * 2 + 1] = val;
			}
			i2s_write(i2s_dev, buf, I2S_BLOCK_SIZE);
		}

		i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);

		/* Stream remaining blocks */
		for (int i = prefill; i < playback_blocks; i++) {
			void *buf;
			k_mem_slab_alloc(&i2s_slab, &buf, K_FOREVER);
			int16_t *out = (int16_t *)buf;

			for (int s = 0; s < I2S_SAMPLES_PER_BLOCK; s++) {
				int16_t val = (sample_idx < total_rec_samples) ?
					      audio_buf[sample_idx++] : 0;
				out[s * 2] = val;
				out[s * 2 + 1] = val;
			}
			ret = i2s_write(i2s_dev, buf, I2S_BLOCK_SIZE);
			if (ret < 0) {
				printk("I2S write failed: %d\n", ret);
				break;
			}
		}

		i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN);

		led_off(led_dev, LED_GREEN);
		gpio_pin_set_dt(&amp_en, 0);
		printk("Done. Press SW3 to record again.\n");
	}

	return 0;
}
