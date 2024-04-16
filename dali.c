
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include <math.h>

#include "cdcacm.h"
#include "dali.h"
#include "platform.h"

/*
 * TE = half period of DALI 1200 Hz
 *
 * - Forward frame: 38TE (35 without stop condition)
 *      - 2 TE      start bit (0, 1)
 *      - 16 * 2TE  data (0 --> (1, 0), 1 --> (0, 1))
 *      - 4 TE      stop condition (1, 1, 1, 1)
 *
 * - Backward frame: 22TE (19 without stop condition)
 *      - 2 TE      start bit (0, 1)
 *      - 8 * 2TE  data (0 --> (1, 0), 1 --> (0, 1))
 *      - 4 TE      stop condition (1, 1, 1, 1)
 *
 * - Settling time (includes stop condition)
 *      - Forward to backward: 5,5 - 10,5ms     --> aim for 14TE
 *      - Before Forward:
 *          - Prio high:        13,5 - 14,7 ms  --> aim for 33TE
 *          - Prio middle high: 14,9 - 16,1 ms  --> aim for 36TE
 *          - Prio middle:      16,3 - 17,7 ms  --> aim for 40TE
 *          - Prio middle low:  17,9 - 19,3 ms  --> aim for 43TE
 *          - Prio low:         19,5 - 21,2 ms  --> aim for 47TE
 *
 */

#define DALI_RX_BUF_LEN 64 /* Allow for some noise */
#define DALI_TX_BUF_LEN 35
#define DALI_FREQ (2 * 1200)
#define DALI_TICK (SYSTICK_FREQ / DALI_FREQ)

volatile uint8_t __rx_state;
volatile uint8_t __rx_buf[DALI_RX_BUF_LEN];
volatile int __rx_at; /* -1 = not started */
volatile uint32_t __rx_last;

volatile char __tx_buf[2];
volatile dali_prio __tx_prio;
volatile uint8_t __tx_len;

volatile char __tx_enc_buf[DALI_TX_BUF_LEN];
volatile uint8_t __tx_enc_len;
volatile int __tx_enc_at; /* -1 = not started */

volatile uint32_t __settling_start_tick;

/* Handlers */

void exti0_isr(void) {
    uint32_t tick = get_systick();

    if (__rx_at > -1 && __rx_at < DALI_RX_BUF_LEN) {
        __rx_buf[__rx_at] = tick - __rx_last;
    }
    __rx_state = !__rx_state;
    __rx_last = tick;
    __rx_at++;

    if (tick > __settling_start_tick)
        __settling_start_tick = tick;

    /* re-enable interrupt */
    exti_reset_request(EXTI0);
    exti_set_trigger(EXTI0,
                     __rx_state ? EXTI_TRIGGER_FALLING : EXTI_TRIGGER_RISING);
}

void tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
        timer_set_counter(TIM2, 0);

        if (__tx_enc_buf[__tx_enc_at]) {
            gpio_set(GPIOB, GPIO1);
        } else {
            gpio_clear(GPIOB, GPIO1);
        }

        __tx_enc_at++;
        if (__tx_enc_at >= __tx_enc_len) {
            __tx_enc_at = -1;
            __tx_enc_len = 0;
            timer_disable_counter(TIM2);
        }

        uint32_t tick = get_systick();
        if (tick > __settling_start_tick)
            __settling_start_tick = tick;
    }
}

/* API */

int dali_init(void) {
    __rx_at = -1;
    __rx_last = 0;
    __tx_len = 0;

    __tx_enc_len = 0;
    __tx_enc_at = -1;

    __settling_start_tick = 0;

    rcc_periph_clock_enable(RCC_GPIOB);

    /* tx setup */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO1);

    /* tx timer setup */
    rcc_periph_clock_enable(RCC_TIM2);
    nvic_enable_irq(NVIC_TIM2_IRQ);
    rcc_periph_reset_pulse(RST_TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_disable_preload(TIM2);
    timer_continuous_mode(TIM2);
    timer_set_prescaler(TIM2, 2 * rcc_apb1_frequency / DALI_FREQ);
    timer_set_period(TIM2, 1);
    timer_set_oc_value(TIM2, TIM_OC1, 1);

    timer_enable_irq(TIM2, TIM_DIER_CC1IE);

    /* rx setup */
    rcc_periph_clock_enable(RCC_AFIO);

    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
    gpio_set(GPIOB, GPIO0);
    __rx_state = 1;

    nvic_enable_irq(NVIC_EXTI0_IRQ);
    exti_select_source(EXTI0, GPIOB);

    exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI0);
    return 0;
}

int dali_read(char *byte1, char *byte2) {
    uint32_t elapsed = get_systick() - __rx_last;
    if (elapsed > 5 * DALI_TICK && __rx_at > -1) {
        uint16_t res = 0;

        uint32_t total = 0;
        int rising = 0;
        int pos = 0;
        for (int i = 0; i < __rx_at; i++) {
            rising = !rising;
            total += __rx_buf[i];

            int total_dali = (int)(0.5 + (double)total / DALI_TICK);
            cdcacm_write_hex(total_dali);
            cdcacm_write(" ", 1);

            if (i == 0) {
                /* start bit */
            } else if (total_dali % 2 == 1) {
                res |= rising << (15 - pos);
                pos++;
            }
        }

        cdcacm_write("=", 1);
        cdcacm_write_hex(pos);
        cdcacm_write("\n", 1);

        __rx_at = -1;

        if (pos == 8) {
            *byte1 = res >> 8;
            return 1;
        } else if (pos == 16) {
            *byte1 = res >> 8;
            *byte2 = res & 0xFF;
            return 2;
        } else {
            return -1;
        }

    } else {
        return 0;
    }
}

int dali_write(char *byte1, char *byte2, dali_prio prio) {
    if (!byte1)
        return -1;

    __tx_buf[0] = *byte1;
    __tx_buf[1] = byte2 ? *byte2 : 0;
    __tx_prio = prio;
    __tx_len = byte2 ? 2 : 1;

    return 0;
}

int dali_main(void) {
    if (__tx_len > 0 && __tx_enc_at == -1) {
        uint32_t settling_time = 0;
        switch (__tx_prio) {
        case DALI_BACKWARD:
            settling_time = 14 * DALI_TICK;
            break;
        case DALI_HIGH:
            settling_time = 33 * DALI_TICK;
            break;
        case DALI_MIDDLE_HIGH:
            settling_time = 36 * DALI_TICK;
            break;
        case DALI_MIDDLE:
            settling_time = 40 * DALI_TICK;
            break;
        case DALI_MIDDLE_LOW:
            settling_time = 43 * DALI_TICK;
            break;
        case DALI_LOW:
            settling_time = 47 * DALI_TICK;
            break;
        }

        if (get_systick() - __settling_start_tick > settling_time) {
            __tx_enc_len = 0;
            __tx_enc_buf[__tx_enc_len++] = 0;
            __tx_enc_buf[__tx_enc_len++] = 1;
            for (int b = 0; b < __tx_len; b++) {
                for (int i = 0; i < 8; i++) {
                    if (__tx_buf[b] >> (7 - i) & 1) {
                        __tx_enc_buf[__tx_enc_len++] = 0;
                        __tx_enc_buf[__tx_enc_len++] = 1;
                    } else {
                        __tx_enc_buf[__tx_enc_len++] = 1;
                        __tx_enc_buf[__tx_enc_len++] = 0;
                    }
                }
            }
            __tx_enc_buf[__tx_enc_len++] = 1;
            __tx_enc_at = 0;
            timer_enable_counter(TIM2);

            __tx_len = 0;
        }
    }
    return 0;
}
