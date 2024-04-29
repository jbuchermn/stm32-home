
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
 * - Forward frame: 38TE (34 without stop condition)
 *      - 2 TE      start bit (0, 1)
 *      - 16 * 2TE  data (0 --> (1, 0), 1 --> (0, 1))
 *      - 4 TE      stop condition (1, 1, 1, 1)
 *
 * - 24bit forward frame: 54TE (50 without stop condition)
 *      - 2 TE      start bit (0, 1)
 *      - 24 * 2TE  data (0 --> (1, 0), 1 --> (0, 1))
 *      - 4 TE      stop condition (1, 1, 1, 1)
 *
 * - Backward frame: 22TE (18 without stop condition)
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

#define DALI_RX_QUEUE_LEN 64
#define DALI_TX_QUEUE_LEN 64

#define DALI_OVERSAMPLE 8
#define DALI_STOP_COND 4 * DALI_OVERSAMPLE
#define DALI_THR_HIGH 7
#define DALI_THR_LOW 1
#define DALI_RX_BUF_LEN 54 * DALI_OVERSAMPLE
#define DALI_TX_BUF_LEN 51
#define DALI_FREQ (2 * 1200)
#define DALI_TICK (SYSTICK_FREQ / DALI_FREQ)

volatile struct dali_msg __rx_queue[DALI_RX_QUEUE_LEN];
volatile int __rx_queue_at;
volatile int __rx_queue_len;

volatile struct dali_msg __tx_queue[DALI_TX_QUEUE_LEN];
volatile int __tx_queue_at;
volatile int __tx_queue_len;

volatile uint8_t __rx_buf[DALI_RX_BUF_LEN];
volatile int __rx_at; /* -1 = not started */

volatile char __tx_enc_buf[DALI_TX_BUF_LEN];
volatile uint8_t __tx_enc_len;
volatile int __tx_enc_at; /* -1 = not started */

volatile uint32_t __settling_start_tick;

/* Pins */
#define DALI_RX_PORT GPIOA
#define DALI_RX_PIN GPIO0
#define DALI_RX_EXTI EXTI0
#define DALI_RX_EXTI_ISR exti0_isr
#define DALI_RX_EXTI_NVIC NVIC_EXTI0_IRQ

#define DALI_TX_PORT GPIOA
#define DALI_TX_PIN GPIO1

/* Handlers */

void DALI_RX_EXTI_ISR(void) {
    if (__rx_at < 0) {
        /* Start RX */
        __rx_at = 0;
        bool val = gpio_get(DALI_RX_PORT, DALI_RX_PIN);
#ifdef DALI_RX_INV
        val = !val;
#endif

        __rx_buf[0] = val;
        __rx_at++;

        timer_enable_counter(TIM2);
    }

    /* re-enable interrupt */
    exti_reset_request(DALI_RX_EXTI);
    exti_set_trigger(DALI_RX_EXTI, EXTI_TRIGGER_BOTH);
}

static bool rx_contains_stop(void) {
    int highs = 0;
    if (__rx_at >= DALI_STOP_COND) {
        for (int i = __rx_at - DALI_STOP_COND; i < __rx_at; i++) {
            if (__rx_buf[i])
                highs++;
        }

        if (highs < DALI_STOP_COND * DALI_THR_HIGH / DALI_OVERSAMPLE)
            return false;

    } else {
        return false;
    }

    return true;
}

static int rx_parse_halfbit(int idx) {
    int c = 0, h = 0;
    for (int i = idx * DALI_OVERSAMPLE; i < (idx + 1) * DALI_OVERSAMPLE; i++) {
        c++;
        h += __rx_buf[i] ? 1 : 0;
    }

    if (c < DALI_OVERSAMPLE)
        return -1;
    if (h <= DALI_THR_LOW)
        return 0;
    if (h >= DALI_THR_HIGH)
        return 1;
    return -1;
}

static int rx_parse_bit(int idx) {
    int b1 = rx_parse_halfbit(idx * 2);
    int b2 = rx_parse_halfbit(idx * 2 + 1);
    if (b1 < 0 || b2 < 0)
        return -1;
    if (b1 == 0 && b2 == 1)
        return 1;
    if (b1 == 1 && b2 == 0)
        return 0;
    return -1;
}

void tim2_isr(void) {
    /* Missing part: Collision detection */
    if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {
        bool rx_done = false;
        bool tx_done = false;
        uint32_t tick = get_systick();

        /* RX */
        if (__rx_at > -1) {
            bool val = gpio_get(DALI_RX_PORT, DALI_RX_PIN);
#ifdef DALI_RX_INV
            val = !val;
#endif
            if (__rx_at >= DALI_RX_BUF_LEN) {
                rx_done = true;
            } else {
                __rx_buf[__rx_at] = val;
            }
            __rx_at++;

            if (rx_contains_stop()) {
                rx_done |= true;

                if (__rx_queue_len < DALI_RX_QUEUE_LEN)
                    __rx_queue[__rx_queue_len].error = true;

                if (rx_parse_bit(0) != 1)
                    goto done;

                uint32_t res = 0;
                int pos;
                for (pos = 0; pos < 24; pos++) {
                    int b = rx_parse_bit(pos + 1);
                    if (b < 0) {
                        if (pos % 8 != 0)
                            /* error */
                            goto done;
                        else
                            /* stop bit */
                            break;
                    }

                    res |= b << (23 - pos);
                }

                if (__rx_queue_len < DALI_RX_QUEUE_LEN) {
                    if (pos == 8) {
                        __rx_queue[__rx_queue_len].data[0] = res >> 16;
                        __rx_queue[__rx_queue_len].len = 1;
                    } else if (pos == 16) {
                        __rx_queue[__rx_queue_len].data[0] = res >> 16;
                        __rx_queue[__rx_queue_len].data[1] = (res >> 8) & 0xFF;
                        __rx_queue[__rx_queue_len].len = 2;
                    } else if (pos == 24) {
                        __rx_queue[__rx_queue_len].data[0] = res >> 16;
                        __rx_queue[__rx_queue_len].data[1] = (res >> 8) & 0xFF;
                        __rx_queue[__rx_queue_len].data[2] = res & 0xFF;
                        __rx_queue[__rx_queue_len].len = 3;
                    }
                    __rx_queue[__rx_queue_len].error = false;
                }

            done:
                __rx_queue_len++;
                __rx_at = -1;
            }

            if (!val && tick > __settling_start_tick)
                __settling_start_tick = tick;
        } else {
            rx_done = true;
        }

        /* TX */
        if (__tx_enc_len > 0) {
            if (__tx_enc_buf[__tx_enc_at / DALI_OVERSAMPLE]) {
#ifdef DALI_TX_INV
                gpio_clear(DALI_TX_PORT, DALI_TX_PIN);
#else
                gpio_set(DALI_TX_PORT, DALI_TX_PIN);
#endif
            } else {
#ifdef DALI_TX_INV
                gpio_set(DALI_TX_PORT, DALI_TX_PIN);
#else
                gpio_clear(DALI_TX_PORT, DALI_TX_PIN);
#endif
            }

            __tx_enc_at++;
            if (__tx_enc_at >= __tx_enc_len * DALI_OVERSAMPLE) {
                __tx_enc_at = -1;
                __tx_enc_len = 0;
                tx_done = true;
            }

            if (tick > __settling_start_tick)
                __settling_start_tick = tick;
        } else {
            tx_done = true;
        }

        timer_clear_flag(TIM2, TIM_SR_CC1IF);
        timer_set_counter(TIM2, 0);

        if (rx_done && tx_done) {
            timer_disable_counter(TIM2);
        }
    }
}

/* API */

int dali_init(void) {
    __tx_queue_at = 0;
    __tx_queue_len = 0;

    __rx_queue_at = 0;
    __rx_queue_len = 0;

    __rx_at = -1;

    __tx_enc_len = 0;
    __tx_enc_at = -1;

    __settling_start_tick = 0;

    rcc_periph_clock_enable(DALI_RX_PORT);
    rcc_periph_clock_enable(DALI_TX_PORT);

    /* timer setup */
    rcc_periph_clock_enable(RCC_TIM2);
    nvic_enable_irq(NVIC_TIM2_IRQ);
    rcc_periph_reset_pulse(RST_TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_disable_preload(TIM2);
    timer_continuous_mode(TIM2);
    timer_set_prescaler(TIM2,
                        2 * rcc_apb1_frequency / DALI_FREQ / DALI_OVERSAMPLE);
    timer_set_period(TIM2, 1);
    timer_set_oc_value(TIM2, TIM_OC1, 1);

    timer_enable_irq(TIM2, TIM_DIER_CC1IE);

    /* tx setup */
    gpio_set_mode(DALI_TX_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, DALI_TX_PIN);
#ifdef DALI_TX_INV
    gpio_clear(DALI_TX_PORT, DALI_TX_PIN);
#else
    gpio_set(DALI_TX_PORT, DALI_TX_PIN);
#endif

    /* rx setup */
    rcc_periph_clock_enable(RCC_AFIO);

    gpio_set_mode(DALI_RX_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
                  DALI_RX_PIN);

    gpio_set(DALI_RX_PORT, DALI_RX_PIN);
    exti_set_trigger(DALI_RX_EXTI, EXTI_TRIGGER_BOTH);

    nvic_enable_irq(DALI_RX_EXTI_NVIC);
    exti_select_source(DALI_RX_EXTI, DALI_RX_PORT);

    exti_enable_request(DALI_RX_EXTI);

    return 0;
}

int dali_read(char *buf) {
    if (__rx_queue_at < __rx_queue_len) {
        int q = __rx_queue_at;

        __rx_queue_at++;
        if (__rx_queue_at == __rx_queue_len)
            __rx_queue_at = __rx_queue_len = 0;

        if (__rx_queue[q].error)
            return -1;

        for (int i = 0; i < __rx_queue[q].len; i++)
            buf[i] = __rx_queue[q].data[i];

        return __rx_queue[q].len;
    }

    return 0;
}

int dali_write(char *buf, int len, dali_prio prio) {
    if (__tx_queue_len >= DALI_TX_QUEUE_LEN)
        return -1;

    __tx_queue[__tx_queue_len].len = len;
    for (int i = 0; i < 3; i++)
        __tx_queue[__tx_queue_len].data[i] = buf[i];
    __tx_queue[__tx_queue_len].prio = prio;

    __tx_queue_len++;
    return 0;
}

int dali_main(void) {
    if (__tx_queue_len > 0 && __tx_queue_at < __tx_queue_len &&
        __tx_enc_at == -1) {
        uint32_t settling_time = 0;
        switch (__tx_queue[__tx_queue_at].prio) {
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
            for (int b = 0; b < __tx_queue[__tx_queue_at].len; b++) {
                for (int i = 0; i < 8; i++) {
                    if (__tx_queue[__tx_queue_at].data[b] >> (7 - i) & 1) {
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

            __tx_queue_at++;
            if (__tx_queue_at == __tx_queue_len)
                __tx_queue_at = __tx_queue_len = 0;
        }
    }

    return 0;
}
