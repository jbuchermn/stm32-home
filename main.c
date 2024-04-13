
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "main.h"
#include "usb_uart.h"

static void clock_setup(void) {
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

  rcc_periph_clock_enable(RCC_GPIOC);
}

static void systick_setup(void) {
  systick_set_reload(rcc_ahb_frequency / 10000); // 10th ms
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_counter_enable();

  systick_interrupt_enable();
}

static void gpio_setup(void) {
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO13);
}

volatile uint32_t __system_clock;
void sys_tick_handler(void) { __system_clock++; }

static void sleep(uint32_t delay) {
  uint32_t wake = __system_clock + delay;
  while (wake > __system_clock)
    ;
}

int main(void) {
  clock_setup();
  gpio_setup();
  cdcacm_init();
  systick_setup();

  gpio_clear(GPIOC, GPIO13);
  while (1) {
    while (!cdcacm_get_configuration())
      sleep(500);

    cdcacm_write_now("hello\n", 6);
    sleep(10000);
    gpio_toggle(GPIOC, GPIO13);
  }

  return 0;
}
