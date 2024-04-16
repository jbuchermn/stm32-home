#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define MAIN_FREQ 100

void sleep_until(uint32_t ticks);

void main_signal_error(void);

#endif
