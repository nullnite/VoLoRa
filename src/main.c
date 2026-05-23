/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/mfd/npm13xx.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(volora, LOG_LEVEL_INF);

/* Audio config */
#define SAMPLE_RATE     16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE (SAMPLE_BIT_WIDTH / 8)
#define NUM_CHANNELS    1

/* Block size for 100ms of mono 16-bit audio at 16kHz = 3200 bytes */
#define BLOCK_SIZE      (BYTES_PER_SAMPLE * (SAMPLE_RATE / 10) * NUM_CHANNELS)
#define BLOCK_COUNT     4

/* I2S needs stereo frames (L+R), so double the block size for TX */
#define I2S_BLOCK_SIZE  (BLOCK_SIZE * 2)

K_MEM_SLAB_DEFINE_STATIC(pdm_mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);
K_MEM_SLAB_DEFINE_STATIC(i2s_mem_slab, I2S_BLOCK_SIZE, BLOCK_COUNT, 4);

/* nPM1300 LED indices */
#define NPM1300_LED_GREEN 1

/* nPM1300 VBUS registers */
#define VBUS_BASE            0x02U
#define VBUS_OFFSET_ILIMUPDATE 0x00U
#define VBUS_OFFSET_ILIM     0x01U
#define VBUS_OFFSET_DETECT   0x05U
#define VBUS_ILIM_500MA      5U

/* MAX98357A SD_MODE enable GPIO (P0.19, active high) */
#define AMP_SD_MODE_NODE DT_NODELABEL(amp_en)
static const struct gpio_dt_spec amp_sd_mode = GPIO_DT_SPEC_GET(AMP_SD_MODE_NODE, gpios);

static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(npm1300_leds));
static const struct device *pmic = DEVICE_DT_GET(DT_NODELABEL(npm1300));
static const struct device *pdm_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
static const struct device *i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));

static int npm1300_enable_vbusout(void)
{
	int ret;

	if (!device_is_ready(pmic)) {
		LOG_ERR("PMIC not ready");
		return -ENODEV;
	}

	ret = mfd_npm13xx_reg_write(pmic, VBUS_BASE, VBUS_OFFSET_ILIM, VBUS_ILIM_500MA);
	if (ret) {
		LOG_ERR("Failed to write VBUS ILIM: %d", ret);
		return ret;
	}

	ret = mfd_npm13xx_reg_write(pmic, VBUS_BASE, VBUS_OFFSET_ILIMUPDATE, 1U);
	if (ret) {
		LOG_ERR("Failed to trigger ILIMUPDATE: %d", ret);
		return ret;
	}

	LOG_INF("nPM1300 VBUSOUT enabled (500mA limit)");
	return 0;
}

static int configure_pdm(void)
{
	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.pcm_rate = SAMPLE_RATE,
		.block_size = BLOCK_SIZE,
		.mem_slab = &pdm_mem_slab,
	};

	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
			.req_num_chan = 1,
			.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT),
		},
	};

	int ret = dmic_configure(pdm_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("PDM configure failed: %d", ret);
		return ret;
	}

	LOG_INF("PDM configured: %u Hz, %u-bit, mono", SAMPLE_RATE, SAMPLE_BIT_WIDTH);
	return 0;
}

static int configure_i2s(void)
{
	struct i2s_config cfg = {
		.word_size = SAMPLE_BIT_WIDTH,
		.channels = 2, /* I2S is always stereo (L+R frames) */
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = SAMPLE_RATE,
		.mem_slab = &i2s_mem_slab,
		.block_size = I2S_BLOCK_SIZE,
		.timeout = 1000,
	};

	int ret = i2s_configure(i2s_dev, I2S_DIR_TX, &cfg);
	if (ret < 0) {
		LOG_ERR("I2S TX configure failed: %d", ret);
		return ret;
	}

	LOG_INF("I2S TX configured: %u Hz, %u-bit, stereo", SAMPLE_RATE, SAMPLE_BIT_WIDTH);
	return 0;
}

/* Gain applied to PDM samples (shift left by this many bits) */
#define PDM_GAIN_SHIFT  4

/* Test tone: 1kHz sine wave (quarter period lookup, 16 samples for 16kHz) */
static const int16_t sine_1khz[] = {
	0, 12539, 23170, 30273, 32767, 30273, 23170, 12539,
	0, -12539, -23170, -30273, -32767, -30273, -23170, -12539
};
#define SINE_TABLE_LEN (sizeof(sine_1khz) / sizeof(sine_1khz[0]))

/**
 * Fill I2S buffer with 1kHz test tone (stereo).
 */
static void fill_test_tone(int16_t *stereo, size_t stereo_samples)
{
	for (size_t i = 0; i < stereo_samples / 2; i++) {
		int16_t sample = sine_1khz[i % SINE_TABLE_LEN];
		stereo[2 * i] = sample;
		stereo[2 * i + 1] = sample;
	}
}

/**
 * Copy mono PDM buffer to stereo I2S buffer with gain.
 */
static void mono_to_stereo(const int16_t *mono, int16_t *stereo, size_t mono_samples)
{
	for (size_t i = 0; i < mono_samples; i++) {
		/* Apply gain with saturation */
		int32_t amplified = (int32_t)mono[i] << PDM_GAIN_SHIFT;
		if (amplified > 32767) amplified = 32767;
		if (amplified < -32768) amplified = -32768;
		stereo[2 * i] = (int16_t)amplified;
		stereo[2 * i + 1] = (int16_t)amplified;
	}
}

int main(void)
{
	int ret;

	LOG_INF("VoLoRa booted - Audio Loopback Test");

	/* Enable VBUSOUT for USB */
	npm1300_enable_vbusout();

	/* Enable MAX98357A amplifier via SD_MODE pin */
	if (!gpio_is_ready_dt(&amp_sd_mode)) {
		LOG_ERR("AMP SD_MODE GPIO not ready");
		return 0;
	}
	ret = gpio_pin_configure_dt(&amp_sd_mode, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to enable AMP SD_MODE: %d", ret);
		return 0;
	}
	LOG_INF("MAX98357A amplifier enabled (SD_MODE HIGH)");

	/* Check devices ready */
	if (!device_is_ready(pdm_dev)) {
		LOG_ERR("PDM device not ready");
		return 0;
	}
	if (!device_is_ready(i2s_dev)) {
		LOG_ERR("I2S device not ready");
		return 0;
	}

	/* Configure I2S */
	ret = configure_i2s();
	if (ret < 0) {
		return 0;
	}

	/* === Read and print PDM mic samples === */
	ret = configure_pdm();
	if (ret < 0) {
		return 0;
	}

	ret = dmic_trigger(pdm_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("PDM start failed: %d", ret);
		return 0;
	}
	LOG_INF("PDM capture started - printing samples");

	/* Discard first block (startup transient) */
	{
		void *discard;
		uint32_t discard_size;
		dmic_read(pdm_dev, 0, &discard, &discard_size, 1000);
		k_mem_slab_free(&pdm_mem_slab, discard);
	}

	/* Print several blocks of PDM samples */
	for (int blk = 0; blk < 10; blk++) {
		void *pdm_buf;
		uint32_t pdm_size;

		ret = dmic_read(pdm_dev, 0, &pdm_buf, &pdm_size, 1000);
		if (ret < 0) {
			LOG_ERR("PDM read failed: %d", ret);
			break;
		}

		int16_t *samples = (int16_t *)pdm_buf;
		int num_samples = pdm_size / sizeof(int16_t);

		/* Print min/max/avg for each block */
		int16_t min_val = 32767, max_val = -32768;
		int32_t sum = 0;
		for (int i = 0; i < num_samples; i++) {
			if (samples[i] < min_val) min_val = samples[i];
			if (samples[i] > max_val) max_val = samples[i];
			sum += samples[i];
		}
		int16_t avg = (int16_t)(sum / num_samples);

		LOG_INF("Block %d: %d samples, min=%d max=%d avg=%d",
			blk, num_samples, min_val, max_val, avg);
		LOG_INF("  [0..7]: %d %d %d %d %d %d %d %d",
			samples[0], samples[1], samples[2], samples[3],
			samples[4], samples[5], samples[6], samples[7]);

		k_mem_slab_free(&pdm_mem_slab, pdm_buf);
	}

	dmic_trigger(pdm_dev, DMIC_TRIGGER_STOP);
	LOG_INF("PDM capture stopped");

	/* === Play 1kHz test tone continuously === */
	LOG_INF("Playing 1kHz test tone CONTINUOUSLY (check BCLK=P1.04, LRCLK=P1.06, DIN=P0.21 with scope)");

	/* Pre-fill 2 blocks with tone */
	for (int i = 0; i < 2; i++) {
		void *i2s_buf;
		ret = k_mem_slab_alloc(&i2s_mem_slab, &i2s_buf, K_MSEC(100));
		if (ret < 0) {
			LOG_ERR("I2S slab alloc failed: %d", ret);
			return 0;
		}
		fill_test_tone((int16_t *)i2s_buf, I2S_BLOCK_SIZE / BYTES_PER_SAMPLE);
		ret = i2s_write(i2s_dev, i2s_buf, I2S_BLOCK_SIZE);
		if (ret < 0) {
			LOG_ERR("I2S write failed: %d", ret);
			k_mem_slab_free(&i2s_mem_slab, i2s_buf);
			return 0;
		}
	}

	ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("I2S TX start failed: %d", ret);
		return 0;
	}
	LOG_INF("I2S TX started");

	/* Continuously feed tone data forever */
	bool led_state = true;
	int block_count = 0;

	while (1) {
		void *i2s_buf;

		ret = k_mem_slab_alloc(&i2s_mem_slab, &i2s_buf, K_MSEC(500));
		if (ret < 0) {
			LOG_ERR("I2S alloc failed: %d (block %d)", ret, block_count);
			break;
		}
		fill_test_tone((int16_t *)i2s_buf, I2S_BLOCK_SIZE / BYTES_PER_SAMPLE);
		ret = i2s_write(i2s_dev, i2s_buf, I2S_BLOCK_SIZE);
		if (ret < 0) {
			LOG_ERR("I2S write failed: %d (block %d)", ret, block_count);
			k_mem_slab_free(&i2s_mem_slab, i2s_buf);
			break;
		}

		block_count++;
		/* Blink LED every 1s (10 blocks * 100ms) */
		if (block_count % 10 == 0) {
			if (device_is_ready(leds)) {
				if (led_state) {
					led_on(leds, NPM1300_LED_GREEN);
				} else {
					led_off(leds, NPM1300_LED_GREEN);
				}
				led_state = !led_state;
			}
			LOG_INF("Tone playing... block %d", block_count);
		}
	}

	LOG_ERR("Tone playback stopped");
	i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
	return 0;
}
