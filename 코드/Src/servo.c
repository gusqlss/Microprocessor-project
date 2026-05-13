/**
 * @file    servo.c
 * @brief   MG996R PWM 제어 구현
 */
#include "servo.h"

/* 각도 [-90, +90] → 펄스 폭 [SERVO_MIN_US, SERVO_MAX_US] 선형 매핑 */
static uint32_t angle_to_us(float angle_deg)
{
    /* -90 ~ +90 클램프 */
    if (angle_deg < -90.0f) angle_deg = -90.0f;
    if (angle_deg > +90.0f) angle_deg = +90.0f;

    /* (angle + 90) / 180 * (max - min) + min */
    float us = ((angle_deg + 90.0f) / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US)
               + SERVO_MIN_US;
    return (uint32_t)(us + 0.5f);
}

/* ════════════════════════════════════════════════════════════ */
void Servo_Init(Servo_t *s, TIM_HandleTypeDef *htim, uint32_t channel,
                float offset_deg, int reverse)
{
    s->htim       = htim;
    s->channel    = channel;
    s->offset_deg = offset_deg;
    s->reverse    = reverse;

    /* 시작은 중립 위치 */
    __HAL_TIM_SET_COMPARE(htim, channel, SERVO_MID_US);
}

/* ════════════════════════════════════════════════════════════ */
void Servo_WriteAngle(Servo_t *s, float angle_deg)
{
    /* 1) 기계적 짐벌 한계 클램프 */
    if (angle_deg >  GIMBAL_ANGLE_LIMIT_DEG) angle_deg =  GIMBAL_ANGLE_LIMIT_DEG;
    if (angle_deg < -GIMBAL_ANGLE_LIMIT_DEG) angle_deg = -GIMBAL_ANGLE_LIMIT_DEG;

    /* 2) 부호 반전 (서보 방향이 반대일 때) */
    if (s->reverse) angle_deg = -angle_deg;

    /* 3) 오프셋 보정 (기계적 중립이 정확히 안 맞을 때) */
    angle_deg += s->offset_deg;

    /* 4) 펄스 폭 계산 후 CCR 갱신 */
    uint32_t us = angle_to_us(angle_deg);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, us);
}

/* ════════════════════════════════════════════════════════════ */
void Servo_StartAll(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
}
