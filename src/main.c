/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* nPM1300 LED indices: 0=Blue, 1=Green, 2=Red */
#define NPM1300_LED_BLUE  0
#define NPM1300_LED_GREEN 1
#define NPM1300_LED_RED   2

static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(npm1300_leds));

int main(void) {
    bool led_state = true;

    printf("VoLoRa booted\n");

    if (!device_is_ready(leds)) {
        printf("nPM1300 LED device not ready\n");
        return 0;
    }

    while (1) {
        if (led_state) {
            led_on(leds, NPM1300_LED_GREEN);
        } else {
            led_off(leds, NPM1300_LED_GREEN);
        }

        led_state = !led_state;
        printf("LED state: %s\n", led_state ? "ON" : "OFF");
        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}
