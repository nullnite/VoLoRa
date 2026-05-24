#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/sys/base64.h>
#include <hal/nrf_pdm.h>

#define SAMPLE_RATE	16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE (SAMPLE_BIT_WIDTH / 8)
#define NUM_CHANNELS	1
#define BLOCK_SIZE	(BYTES_PER_SAMPLE * (SAMPLE_RATE / 50) * NUM_CHANNELS)
#define BLOCK_COUNT	8
#define READ_TIMEOUT_MS	1000
#define RECORD_SECONDS	5
#define TOTAL_BYTES	(SAMPLE_RATE * RECORD_SECONDS * BYTES_PER_SAMPLE * NUM_CHANNELS)

K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

/* Audio buffer in RAM - record first, send after */
static uint8_t audio_buf[TOTAL_BYTES];

/* Base64 output buffer */
static char b64_buf[((BLOCK_SIZE + 2) / 3) * 4 + 1];

int main(void)
{
	const struct device *dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	int ret;

	printk("VoLoRa PDM Mic Test\n");
	printk("Sample rate: %d Hz, %d-bit, %d ch, %d s\n",
	       SAMPLE_RATE, SAMPLE_BIT_WIDTH, NUM_CHANNELS, RECORD_SECONDS);

	if (!device_is_ready(dmic_dev)) {
		printk("DMIC device not ready\n");
		return 0;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab = &mem_slab,
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
			.req_num_chan = NUM_CHANNELS,
			.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT),
		},
	};
	stream.pcm_rate = SAMPLE_RATE;
	stream.block_size = BLOCK_SIZE;

	ret = dmic_configure(dmic_dev, &cfg);
	if (ret < 0) {
		printk("DMIC configure failed: %d\n", ret);
		return 0;
	}

	/* Set PDM gain to maximum (+20 dB) for -26 dBFS sensitivity mic */
	NRF_PDM0_S->GAINL = 0x50;
	NRF_PDM0_S->GAINR = 0x50;

	printk("UICR.NFCPINS=0x%08x\n", NRF_UICR_S->NFCPINS);
	printk("Gain: +20dB. Speak into mic during recording!\n");
	printk("Recording starts in 3 seconds...\n");
	k_msleep(3000);

	printk("Recording...\n");

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		printk("DMIC start failed: %d\n", ret);
		return 0;
	}

	/* Record to RAM */
	size_t offset = 0;
	int total_blocks = TOTAL_BYTES / BLOCK_SIZE;

	for (int i = 0; i < total_blocks; i++) {
		void *buffer;
		uint32_t size;

		ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT_MS);
		if (ret < 0) {
			break;
		}

		size_t copy_len = MIN(size, TOTAL_BYTES - offset);
		memcpy(&audio_buf[offset], buffer, copy_len);
		offset += copy_len;

		k_mem_slab_free(&mem_slab, buffer);
	}

	dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);

	/* Signal stats */
	int16_t *samples = (int16_t *)audio_buf;
	int total_samples = offset / 2;
	int16_t min_val = 0, max_val = 0;
	for (int i = 0; i < total_samples; i++) {
		if (samples[i] < min_val) min_val = samples[i];
		if (samples[i] > max_val) max_val = samples[i];
	}
	printk("Recording done: %u bytes, min=%d max=%d\n",
	       (unsigned int)offset, min_val, max_val);
	k_msleep(100);

	/* Send recorded audio as base64 */
	printk("<<PDM_START rate=%d bits=%d channels=%d>>\n",
	       SAMPLE_RATE, SAMPLE_BIT_WIDTH, NUM_CHANNELS);

	for (size_t pos = 0; pos < offset; pos += BLOCK_SIZE) {
		size_t chunk = MIN(BLOCK_SIZE, offset - pos);
		size_t b64_len;

		ret = base64_encode(b64_buf, sizeof(b64_buf), &b64_len,
				    &audio_buf[pos], chunk);
		if (ret == 0) {
			b64_buf[b64_len] = '\0';
			printk("%s\n", b64_buf);
		}

		k_msleep(5);
	}

	printk("<<PDM_END>>\n");
	printk("Transfer complete\n");

	return 0;
}
