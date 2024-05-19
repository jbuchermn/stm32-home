#include "led.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "cdcacm.h"

static bool __led_state[LED_COUNT];

static void setup_timer(uint32_t timer) {
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_set_prescaler(timer, 2 * rcc_apb1_frequency / PWM_FREQ / LED_MAX);
    timer_enable_preload(timer);
    timer_set_period(timer, LED_MAX);
    timer_set_counter(timer, 0);

    timer_set_oc_value(timer, TIM_OC1, 0);
    timer_enable_oc_preload(timer, TIM_OC1);
    timer_enable_oc_preload(timer, TIM_OC2);
    timer_enable_oc_preload(timer, TIM_OC3);
    timer_enable_oc_preload(timer, TIM_OC4);

    timer_set_oc_mode(timer, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(timer, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_mode(timer, TIM_OC3, TIM_OCM_PWM1);
    timer_set_oc_mode(timer, TIM_OC4, TIM_OCM_PWM1);

    timer_enable_break_main_output(timer);

    timer_enable_oc_output(timer, TIM_OC1);
    timer_enable_oc_output(timer, TIM_OC2);
    timer_enable_oc_output(timer, TIM_OC3);
    timer_enable_oc_output(timer, TIM_OC4);

    timer_continuous_mode(timer);
    timer_generate_event(timer, TIM_EGR_UG);
}

int led_init(void) {
    for (int i = 0; i < LED_COUNT; i++)
        __led_state[i] = false;

    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_TIM4);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_AFIO);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO_TIM1_CH1 | GPIO_TIM1_CH2 | GPIO_TIM1_CH3 |
                      GPIO_TIM1_CH4 | GPIO_TIM3_CH1 | GPIO_TIM3_CH2);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO_TIM3_CH3 | GPIO_TIM3_CH4 | GPIO_TIM4_CH1 |
                      GPIO_TIM4_CH2 | GPIO_TIM4_CH3 | GPIO_TIM4_CH4);

    setup_timer(TIM1);
    setup_timer(TIM3);
    setup_timer(TIM4);

    return 0;
}

static void __do_set(uint32_t timer, int id, int offset, uint16_t value) {
    bool timer_is_on = false;
    bool timer_will_be_on = false;

    for (int i = offset, j = 0; j < 4; i++, j++)
        if (__led_state[i]) {
            timer_is_on = true;
            break;
        }
    __led_state[id] = value != 0;
    for (int i = offset, j = 0; j < 4; i++, j++)
        if (__led_state[i]) {
            timer_will_be_on = true;
            break;
        }

    timer_set_oc_value(timer, TIM_OC1 + (id - offset) * 2, value);
    if (!timer_is_on && timer_will_be_on) {
        timer_enable_counter(timer);
    } else if (timer_is_on && !timer_will_be_on) {
        timer_disable_counter(timer);
    }
}

void led_set(int number, uint16_t value) {
    if (number >= LED_COUNT)
        return;

    if (number < 4)
        __do_set(TIM1, number, 0, value);
    else if (number < 8)
        __do_set(TIM3, number, 4, value);
    else
        __do_set(TIM4, number, 8, value);
}
