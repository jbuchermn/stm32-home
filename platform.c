#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "platform.h"

volatile uint32_t __systick;

/* Handlers */

void sys_tick_handler(void) { __systick++; }

/* API */

int platform_init(void) {
    /* clock */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    if (!systick_set_frequency(SYSTICK_FREQ, rcc_ahb_frequency))
        return -1;
    systick_counter_enable();

    /* led */
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO13);
    gpio_clear(GPIOC, GPIO13);

    /* enable interrupts */
    systick_interrupt_enable();

    return 0;
}

void set_led(bool on) {
    if (!on) {
        gpio_set(GPIOC, GPIO13);
    } else {
        gpio_clear(GPIOC, GPIO13);
    }
}

uint32_t get_systick(void) { return __systick; }
