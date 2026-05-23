/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/mfd/npm13xx.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(volora, LOG_LEVEL_INF);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* nPM1300 LED indices: 0=Blue, 1=Green, 2=Red */
#define NPM1300_LED_BLUE  0
#define NPM1300_LED_GREEN 1
#define NPM1300_LED_RED   2

/* nPM1300 VBUS registers */
#define VBUS_BASE            0x02U
#define VBUS_OFFSET_ILIMUPDATE 0x00U
#define VBUS_OFFSET_ILIM     0x01U
#define VBUS_OFFSET_DETECT   0x05U
#define VBUS_OFFSET_STATUS   0x07U

/* VBUS current limit index: 5 = 500mA */
#define VBUS_ILIM_500MA      5U

static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(npm1300_leds));
static const struct device *pmic = DEVICE_DT_GET(DT_NODELABEL(npm1300));

/**
 * Enable nPM1300 VBUSOUT by setting current limit and triggering update.
 * This makes VBUS available on VBUSOUT pin for nRF5340 USB detection.
 */
static int npm1300_enable_vbusout(void)
{
    int ret;
    uint8_t detect = 0;

    if (!device_is_ready(pmic)) {
        LOG_ERR("PMIC not ready");
        return -ENODEV;
    }

    /* Check if VBUS is detected by nPM1300 */
    ret = mfd_npm13xx_reg_read(pmic, VBUS_BASE, VBUS_OFFSET_DETECT, &detect);
    if (ret) {
        LOG_ERR("Failed to read VBUS detect: %d", ret);
        return ret;
    }
    LOG_INF("nPM1300 VBUS detect: 0x%02x", detect);

    /* Set active current limit to 500mA */
    ret = mfd_npm13xx_reg_write(pmic, VBUS_BASE, VBUS_OFFSET_ILIM, VBUS_ILIM_500MA);
    if (ret) {
        LOG_ERR("Failed to write VBUS ILIM: %d", ret);
        return ret;
    }

    /* Trigger ILIM update to apply immediately */
    ret = mfd_npm13xx_reg_write(pmic, VBUS_BASE, VBUS_OFFSET_ILIMUPDATE, 1U);
    if (ret) {
        LOG_ERR("Failed to trigger ILIMUPDATE: %d", ret);
        return ret;
    }

    LOG_INF("nPM1300 VBUSOUT enabled (500mA limit)");
    return 0;
}

int main(void) {
    bool led_state = true;

    LOG_INF("VoLoRa booted");

    /* Enable VBUSOUT so nRF5340 can detect USB VBUS */
    npm1300_enable_vbusout();

    if (!device_is_ready(leds)) {
        LOG_ERR("nPM1300 LED device not ready");
        while (1) {
            k_msleep(1000);
        }
    }

    while (1) {
        if (led_state) {
            led_on(leds, NPM1300_LED_GREEN);
        } else {
            led_off(leds, NPM1300_LED_GREEN);
        }

        led_state = !led_state;
        LOG_INF("LED state: %s", led_state ? "ON" : "OFF");
        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}
