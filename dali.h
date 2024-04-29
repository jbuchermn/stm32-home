#ifndef DALI_H
#define DALI_H

#include <libopencm3/stm32/gpio.h>

#define DALI_TX_INV
#define DALI_RX_INV

typedef enum {
    DALI_BACKWARD,
    DALI_LOW,
    DALI_MIDDLE_LOW,
    DALI_MIDDLE,
    DALI_MIDDLE_HIGH,
    DALI_HIGH
} dali_prio;

struct dali_msg {
    int len;
    char data[3];
    dali_prio prio;
    bool error;
};

int dali_init(void);
int dali_main(void);

int dali_write(char *buf, int len, dali_prio prio);

/* number of bytes, negative = corrupt packet */
int dali_read(char *buf);

#endif
