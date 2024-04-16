#include <stdlib.h>
#include <string.h>

#include "main.h"

#include "cdcacm.h"
#include "dali.h"
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
    dali_main();

    char dali1, dali2;
    int len;
    if ((len = dali_read(&dali1, &dali2))) {
        cdcacm_write("[DALI] RX ", 10);
        if (len < 0) {
            cdcacm_write("ERR", 3);
        } else {
            cdcacm_write_hex(dali1 >> 4);
            cdcacm_write_hex(dali1 & 15);
            if (len == 2) {
                cdcacm_write_hex(dali2 >> 4);
                cdcacm_write_hex(dali2 & 15);
            }
        }
        cdcacm_write("\n", 1);

        /* TODO: Handle DALI command */
    }

    uint16_t line;
    /* TODO: Send multiple commands in one line, send single-line commands */
    if ((line = cdcacm_len_line()) > 0) {
        char *buf;
        cdcacm_read(&buf, 6);

        if (buf[0] == 'd' && line >= 5) {
            char temp[5];
            strncpy(temp, buf + 1, 4);
            temp[4] = 0;
            long tmp = strtol(temp, 0, 16);

            char byte1 = tmp >> 8;
            char byte2 = tmp & 0xFF;

            dali_write(&byte1, &byte2, DALI_MIDDLE_LOW /* TODO */);
            cdcacm_write("[DALI] TX ", 10);
            cdcacm_write_hex(byte1 >> 4);
            cdcacm_write_hex(byte1 & 15);
            cdcacm_write_hex(byte2 >> 4);
            cdcacm_write_hex(byte2 & 15);
            cdcacm_write("\n", 1);

        } else if (buf[0] == 'r' && line >= 5) {
            char temp[5];
            strncpy(temp, buf + 1, 4);
            temp[4] = 0;
            long tmp = strtol(temp, 0, 16);

            char byte1 = tmp >> 8;
            char byte2 = tmp & 0xFF;
            cdcacm_write("[DALI] RX ", 10);
            cdcacm_write_hex(byte1 >> 4);
            cdcacm_write_hex(byte1 & 15);
            cdcacm_write_hex(byte2 >> 4);
            cdcacm_write_hex(byte2 & 15);
            cdcacm_write("\n", 1);

            /* TODO: Handle DALI command */
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
    dali_init();

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
