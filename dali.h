#ifndef DALI_H
#define DALI_H

#include <libopencm3/stm32/gpio.h>

typedef enum {
    DALI_BACKWARD,
    DALI_LOW,
    DALI_MIDDLE_LOW,
    DALI_MIDDLE,
    DALI_MIDDLE_HIGH,
    DALI_HIGH
} dali_prio;

int dali_init(void);

int dali_main(void);

int dali_read(char *byte1, char *byte2); /* number of bytes, negative = corrupt packet */
int dali_write(char *byte1, char *byte2, dali_prio prio);

#endif