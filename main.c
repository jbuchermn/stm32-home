#include <stdlib.h>
#include <string.h>

#include "main.h"

#include "config.h"

#include "app.h"
#include "cdcacm.h"
#include "dali.h"
#include "knx.h"
#include "led.h"
#include "platform.h"

static int error;

void main_signal_error(void) { error = 1; }

static void __sleep_until(uint32_t ticks) {
    while (get_systick() > ticks)
        ;
    while (get_systick() < ticks)
        ;
}

static void main_tick(void) {
    cdcacm_main();
    knx_main();
    dali_main();
    app_main();

    char dali[3];
    int len;
    if ((len = dali_read(dali))) {
#ifdef CONFIG_ECHO_DALI
        cdcacm_write("[DALI] RX ", 10);
        if (len < 0) {
            cdcacm_write("ERR", 3);
        } else {
            for (int i = 0; i < len; i++) {
                cdcacm_write_hex(dali[i], 2);
            }
        }
        cdcacm_write("\n", 1);
#endif

        app_handle_dali(dali, len);
    }

    struct knx_telegram telegram;
    if (knx_read(&telegram)) {
#ifdef CONFIG_ECHO_KNX
        if (telegram.is_echo) {
            cdcacm_write("[KNX] TX ", 10);
        } else {
            cdcacm_write("[KNX] RX ", 10);
        }
        cdcacm_write_hex(telegram.source_address, 4);
        cdcacm_write(" -> ", 4);
        cdcacm_write_hex(telegram.target_address, 4);
        cdcacm_write(": ", 2);
        cdcacm_write_hex(telegram.apci, 4);
        if (telegram.ack) {
            cdcacm_write(" (ACK)", 6);
        }
        cdcacm_write("\n", 1);
#endif
        app_handle_knx(&telegram);
    }

    uint16_t line;
    if ((line = cdcacm_len_line()) > 1) {
        char *buf;
        cdcacm_read(&buf, line);

        int at = 0;
        for (int i = 0; i <= line; i++) {
            if (i == line || buf[i] == ',') {
                if (i > at)
                    app_usb_command(buf + at, i - at);
                at = i + 1;
            }
        }
    }
}

void sleep_until(uint32_t ticks) {
    uint32_t t;
    while ((t = get_systick()) > ticks) {
        main_tick();
        __sleep_until(t + SYSTICK_FREQ / MAIN_FREQ);
    }
    while ((t = get_systick()) < ticks) {
        main_tick();
        __sleep_until(t + SYSTICK_FREQ / MAIN_FREQ);
    }
}

int main(void) {
    platform_init();
    cdcacm_init();
    knx_init();
    dali_init();
    led_init();
    app_init();

    uint32_t t = get_systick();
    while (1) {
        t += SYSTICK_FREQ;
        sleep_until(t);

        set_led(1);
        if (error) {
            sleep_until(t + SYSTICK_FREQ * 0.9);
        } else {
            sleep_until(t + SYSTICK_FREQ * 0.1);
        }
        set_led(0);
    }

    return 0;
}
