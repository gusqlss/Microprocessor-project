/**
 * @file    bts7960.c
 */
#include "bts7960.h"

void BTS7960_Init(BTS7960_t *m,
                  TIM_HandleTypeDef *htim_rpwm, uint32_t ch_rpwm,
                  TIM_HandleTypeDef *htim_lpwm, uint32_t ch_lpwm,
                  uint32_t arr, int reverse)
{
    m->htim_rpwm = htim_rpwm;
    m->ch_rpwm   = ch_rpwm;
    m->htim_lpwm = htim_lpwm;
    m->ch_lpwm   = ch_lpwm;
    m->arr       = arr;
    m->reverse   = reverse;

    /* PWM 시작 (CCR=0 이므로 무회전) */
    __HAL_TIM_SET_COMPARE(htim_rpwm, ch_rpwm, 0);
    __HAL_TIM_SET_COMPARE(htim_lpwm, ch_lpwm, 0);
    HAL_TIM_PWM_Start(htim_rpwm, ch_rpwm);
    HAL_TIM_PWM_Start(htim_lpwm, ch_lpwm);
}

void BTS7960_SetSpeed(BTS7960_t *m, float speed)
{
    /* 1) 클램프 */
    if (speed >  1.0f) speed =  1.0f;
    if (speed < -1.0f) speed = -1.0f;

    /* 2) 방향 반전 (조립 시 모터 +선/-선이 반대로 됐을 때 보정용) */
    if (m->reverse) speed = -speed;

    /* 3) duty 계산 */
    uint32_t duty = (uint32_t)(m->arr * (speed >= 0 ? speed : -speed));

    /* 4) 방향에 맞춰 R/L PWM 분배 (절대 둘 다 동시에 PWM 주지 말 것) */
    if (speed > 0.0f) {
        __HAL_TIM_SET_COMPARE(m->htim_rpwm, m->ch_rpwm, duty);
        __HAL_TIM_SET_COMPARE(m->htim_lpwm, m->ch_lpwm, 0);
    } else if (speed < 0.0f) {
        __HAL_TIM_SET_COMPARE(m->htim_rpwm, m->ch_rpwm, 0);
        __HAL_TIM_SET_COMPARE(m->htim_lpwm, m->ch_lpwm, duty);
    } else {
        /* speed == 0 → 정지 */
        __HAL_TIM_SET_COMPARE(m->htim_rpwm, m->ch_rpwm, 0);
        __HAL_TIM_SET_COMPARE(m->htim_lpwm, m->ch_lpwm, 0);
    }
}

void BTS7960_Stop(BTS7960_t *m)
{
    BTS7960_SetSpeed(m, 0.0f);
}
