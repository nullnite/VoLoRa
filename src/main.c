#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>

#define LORA_NODE DT_ALIAS(lora0)
#define FREQUENCY 868000000
#define TX_POWER 14
#define TX_INTERVAL K_SECONDS(10)

static const struct device* lora_dev = DEVICE_DT_GET(LORA_NODE);

static struct lora_modem_config cfg = {
    .frequency = FREQUENCY,
    .bandwidth = BW_125_KHZ,
    .datarate = SF_7,
    .coding_rate = CR_4_5,
    .preamble_len = 8,
    .tx_power = TX_POWER,
};

static void enter_rx(void);

static void rx_callback(const struct device* dev, uint8_t* data,
                        uint16_t size, int16_t rssi, int8_t snr,
                        void* user_data) {
    char buf[128];
    size = MIN(size, sizeof(buf) - 1);
    memcpy(buf, data, size);
    buf[size] = '\0';
    printf("RX (%u B) RSSI=%d dBm SNR=%d dB: \"%s\"\n",
           size, rssi, snr, buf);
    /* Driver automatically restarts RX — no need to re-arm here */
}

static void enter_rx(void) {
    int ret;

    cfg.tx = false;
    ret = lora_config(lora_dev, &cfg);
    if (ret < 0) {
        printf("RX config failed: %d\n", ret);
        return;
    }
    ret = lora_recv_async(lora_dev, rx_callback, NULL);
    if (ret < 0) {
        printf("lora_recv_async failed: %d\n", ret);
    }
}

int main(void) {
    int ret;
    int seq = 0;
    char buf[64];

    if (!device_is_ready(lora_dev)) {
        printf("LoRa device not ready\n");
        return -1;
    }
    printf("LLCC68 ready\n");

    enter_rx();

    while (1) {
        k_sleep(TX_INTERVAL);

        /* Cancel async RX before reconfiguring for TX */
        lora_recv_async(lora_dev, NULL, NULL);

        /* Switch to TX */
        cfg.tx = true;
        ret = lora_config(lora_dev, &cfg);
        if (ret < 0) {
            printf("TX config failed: %d\n", ret);
            enter_rx();
            continue;
        }

        snprintf(buf, sizeof(buf), "LLCC68 test #%d", seq++);
        ret = lora_send(lora_dev, (uint8_t*)buf, strlen(buf));
        if (ret < 0) {
            printf("TX failed: %d\n", ret);
        } else {
            printf("TX: \"%s\"\n", buf);
        }

        /* Return to RX as fast as possible after TX */
        enter_rx();
    }

    return 0;
}
