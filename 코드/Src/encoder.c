/**
 * @file    encoder.c
 */
#include "encoder.h"

void Encoder_Init(Encoder_t *e,
                  uint16_t pin_fl, uint16_t pin_fr,
                  uint16_t pin_rl, uint16_t pin_rr)
{
    e->exti_pin[0] = pin_fl;
    e->exti_pin[1] = pin_fr;
    e->exti_pin[2] = pin_rl;
    e->exti_pin[3] = pin_rr;
    for (int i = 0; i < ENC_COUNT; i++) {
        e->count[i]     = 0;
        e->direction[i] = +1;
    }
}

void Encoder_SetDirection(Encoder_t *e, int idx, float motor_speed)
{
    if (idx < 0 || idx >= ENC_COUNT) return;
    if      (motor_speed > 0.0f) e->direction[idx] = +1;
    else if (motor_speed < 0.0f) e->direction[idx] = -1;
    /* speed == 0이면 직전 방향 유지 (관성으로 조금 더 굴러갈 수 있음) */
}

int32_t Encoder_GetCount(Encoder_t *e, int idx)
{
    if (idx < 0 || idx >= ENC_COUNT) return 0;
    /* int32_t 단일 변수 읽기는 ARM Cortex-M에서 atomic이라 별도 보호 불필요 */
    return e->count[idx];
}

void Encoder_Reset(Encoder_t *e)
{
    for (int i = 0; i < ENC_COUNT; i++) e->count[i] = 0;
}

void Encoder_OnInterrupt(Encoder_t *e, uint16_t GPIO_Pin)
{
    /* 4채널 중 일치하는 핀 찾기 */
    for (int i = 0; i < ENC_COUNT; i++) {
        if (GPIO_Pin == e->exti_pin[i]) {
            e->count[i] += e->direction[i];
            return;
        }
    }
}
