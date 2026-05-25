#ifndef __BTS7960_H
#define __BTS7960_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim_rpwm; // PWM용 타이머
    uint32_t           ch_rpwm;   // PWM 채널
    GPIO_TypeDef      *gpio_lpwm; // 방향 제어용 GPIO 포트
    uint16_t           pin_lpwm;  // 방향 제어용 GPIO 핀
    uint32_t           arr;       // 타이머 ARR
    int                reverse;   // 회전 방향 반전 플래그
} BTS7960_t;

void BTS7960_Init(BTS7960_t *m,
                  TIM_HandleTypeDef *htim_rpwm, uint32_t ch_rpwm,
                  GPIO_TypeDef *gpio_lpwm, uint16_t pin_lpwm,
                  uint32_t arr, int reverse);

void BTS7960_SetSpeed(BTS7960_t *m, float speed);
void BTS7960_Stop(BTS7960_t *m);

#endif
