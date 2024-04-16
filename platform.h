#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#define SYSTICK_FREQ 24000

int platform_init(void);

void set_led(bool on);
uint32_t get_systick(void);

#endif
