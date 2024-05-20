#include <stdlib.h>
#include <string.h>

#include "app.h"
#include "cdcacm.h"
#include "config.h"
#include "dali.h"
#include "led.h"
#include "main.h"
#include "platform.h"

#define DIM_ITERATIONS (CONFIG_DIM_MS * MAIN_FREQ / 1000)

static uint16_t __led_target[LED_COUNT];
static uint16_t __led_current[LED_COUNT];
static uint16_t __led_speed[LED_COUNT];

static void led_set_smooth(int number, uint16_t value) {
    __led_target[number] = value;
    uint16_t delta = value < __led_current[number]
                         ? __led_current[number] - value
                         : value - __led_current[number];
    __led_speed[number] = delta / DIM_ITERATIONS;
}

void app_init(void) {
    for (int i = 0; i < LED_COUNT; i++) {
        __led_target[i] = 0;
        __led_current[i] = 0;
        __led_speed[i] = 0;
        led_set(i, 0);
    }
}

void app_handle_dali(char *dali, int len) {
    /* POC */
    if (len == 2 && dali[0] == 0) {
        led_set_smooth(0, dali[1] * LED_MAX / 255);
    }
}

void app_handle_knx(struct knx_telegram *telegram) {
    /* POC */
    if (telegram->target_is_group_address &&
        telegram->target_address == 0x0003) {
        led_set_smooth(0, (telegram->apci & 0x01) * LED_MAX);
    }
}
bool app_knx_is_addressed(struct knx_telegram *telegram) {
    return telegram->target_is_group_address &&
           telegram->target_address == 0x0003;
}

void app_main(void) {
    for (int i = 0; i < LED_COUNT; i++) {
        if (__led_target[i] < __led_current[i]) {
            int new = (int)__led_current[i] - (int)__led_speed[i];
            if (new < __led_target[i])
                new = __led_target[i];
            led_set(i, new);
            __led_current[i] = new;

        } else if (__led_target[i] > __led_current[i]) {
            int new = (int)__led_current[i] + (int)__led_speed[i];
            if (new > __led_target[i])
                new = __led_target[i];
            led_set(i, new);
            __led_current[i] = new;
        }
    }
}

void app_usb_command(char *cmd, int len) {
    /* command t: send dummy test  via DALI */
    if (cmd[0] == 't') {
        char dali_buf[2];
        dali_buf[0] = 0x01;
        dali_buf[1] = 0x92;
        dali_write(dali_buf, 2, DALI_MIDDLE_LOW);
        cdcacm_write("[DALI] TX 0192\n", 15);
    }

    /* command T: send dummy test  via KNX */
    else if (cmd[0] == 'T') {
        struct knx_telegram test;
        /* UDP defaults */
        test.repeated = false;
        test.priority = 0b11;
        test.routing_counter = 0b110;
        test.sequence = -1;

        /* content */
        test.source_address = 0b0001000100001110;
        test.target_address = 0x0006;
        test.target_is_group_address = true;
        test.apci = 0x081;
        test.payload_len = 0;

        knx_write(&test);
        cdcacm_write("[KNX] TX Test\n", 15);
    }

    /* command d: send DALI with priority (BLlmhH) - e.g. dm0192 */
    else if (cmd[0] == 'd') {
        if (len < 4) {
            cdcacm_write("Error\n", 6);
            return;
        }
        dali_prio prio = DALI_MIDDLE_LOW;
        switch (cmd[1]) {
        case 'B':
            prio = DALI_BACKWARD;
            break;
        case 'L':
            prio = DALI_LOW;
            break;
        case 'l':
            prio = DALI_MIDDLE_LOW;
            break;
        case 'm':
            prio = DALI_MIDDLE;
            break;
        case 'h':
            prio = DALI_MIDDLE_HIGH;
            break;
        case 'H':
            prio = DALI_HIGH;
            break;
        }

        char dali_buf[3];
        int bytes = (len - 2) / 2;
        if (bytes > 3)
            bytes = 3;
        cdcacm_write("[DALI] TX ", 10);
        for (int i = 0; i < bytes; i++) {
            char temp[3];
            strncpy(temp, cmd + 2 + 2 * i, 2);
            temp[2] = 0;
            dali_buf[i] = strtol(temp, 0, 16);

            cdcacm_write_hex(dali_buf[i], 2);
        }

        cdcacm_write("\n", 1);
        dali_write(dali_buf, bytes, prio);

    }

    /* command r: receive DALI - e.g. r0192 */
    else if (cmd[0] == 'r') {
        char dali_buf[3];
        int bytes = (len - 2) / 2;
        if (bytes > 3)
            bytes = 3;
        cdcacm_write("[DALI] -- ", 10);
        for (int i = 0; i < bytes; i++) {
            char temp[3];
            strncpy(temp, cmd + 1 + 2 * i, 2);
            temp[2] = 0;
            dali_buf[i] = strtol(temp, 0, 16);

            cdcacm_write_hex(dali_buf[i], 2);
        }

        app_handle_dali(dali_buf, bytes);
        cdcacm_write("\n", 1);
    }

    /* command l: set LED n value v, lnnvv, e.g. l01ff */
    else if (cmd[0] == 'l') {
        if (len < 5) {
            cdcacm_write("Error\n", 6);
            return;
        }
        char temp[3];
        strncpy(temp, cmd + 1, 2);
        temp[2] = 0;
        int number = strtol(temp, 0, 16);
        strncpy(temp, cmd + 3, 2);
        int value = strtol(temp, 0, 16);

        if (number >= LED_COUNT) {
            cdcacm_write("Error\n", 6);
            return;
        }

        cdcacm_write("[LED] ", 6);
        cdcacm_write_hex(number, 2);
        cdcacm_write(" to ", 4);
        cdcacm_write_hex(value, 2);
        cdcacm_write("\n", 1);

        led_set_smooth(number, value * LED_MAX / 256);
    }
}
