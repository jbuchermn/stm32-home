#ifndef LED_H
#define LED_H

#include <stdint.h>

/*
 * LED0:  TIM1_CH1 PA8
 * LED1:  TIM1_CH2 PA9
 * LED2:  TIM1_CH3 PA10
 * LED3:  TIM1_CH4 PA11
 * LED4:  TIM3_CH1 PA6
 * LED5:  TIM3_CH2 PA7
 * LED6:  TIM3_CH3 PB0
 * LED7:  TIM3_CH4 PB1
 * LED8:  TIM4_CH1 PB6
 * LED9:  TIM4_CH2 PB7
 * LED10: TIM4_CH3 PB8
 * LED11: TIM4_CH4 PB9
 *
 * Used by DALI (only one channel required, three could be used if speed synced)
 * TIM2_CH1 PA15
 * TIM2_CH2 PA1
 * TIM2_CH3 PA2
 * TIM2_CH4 PA3
 */

#define LED_COUNT 12
#define LED_MAX 65535
#define PWM_FREQ 1000

int led_init(void);

void led_set(int number, uint16_t value);

#endif
