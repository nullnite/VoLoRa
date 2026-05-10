#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>

#define LORA_NODE     DT_ALIAS(lora0)
#define FREQUENCY     868000000  /* Hz - change to 915000000 for US */
#define TX_POWER      14         /* dBm */
#define PAYLOAD       "LLCC68 test"

static const struct device *lora_dev = DEVICE_DT_GET(LORA_NODE);

static struct lora_modem_config cfg = {
    .frequency   = FREQUENCY,
    .bandwidth   = BW_125_KHZ,
    .datarate    = SF_7,
    .coding_rate = CR_4_5,
    .preamble_len = 8,
    .tx_power    = TX_POWER,
    .tx          = true,
};

int main(void)
{
    int ret;
    uint8_t buf[64];
    int16_t rssi;
    int8_t  snr;
    int seq = 0;

    if (!device_is_ready(lora_dev)) {
        printf("LoRa device not ready\n");
        return -1;
    }
    printf("LLCC68 ready\n");

    /* TX: send a packet, then flip to RX to listen for an echo */
    while (1) {
        /* --- TX --- */
        cfg.tx = true;
        ret = lora_config(lora_dev, &cfg);
        if (ret < 0) {
            printf("TX config failed: %d\n", ret);
            k_msleep(1000);
            continue;
        }

        snprintf(buf, sizeof(buf), "%s #%d", PAYLOAD, seq++);
        ret = lora_send(lora_dev, buf, strlen(buf));
        if (ret < 0) {
            printf("TX failed: %d\n", ret);
        } else {
            printf("TX: \"%s\"\n", buf);
        }

        /* --- RX: listen for 5 s for a reply --- */
        cfg.tx = false;
        ret = lora_config(lora_dev, &cfg);
        if (ret < 0) {
            printf("RX config failed: %d\n", ret);
            k_msleep(1000);
            continue;
        }

        ret = lora_recv(lora_dev, buf, sizeof(buf) - 1, K_SECONDS(5), &rssi, &snr);
        if (ret > 0) {
            buf[ret] = '\0';
            printf("RX (%d B) RSSI=%d dBm SNR=%d dB: \"%s\"\n",
                   ret, rssi, snr, buf);
        } else if (ret == -EAGAIN) {
            printf("RX timeout\n");
        } else {
            printf("RX error: %d\n", ret);
        }

        k_msleep(500);
    }

    return 0;
}
