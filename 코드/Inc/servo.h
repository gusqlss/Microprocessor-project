/**
 * @file    servo.h
 * @brief   MG996R 서보 제어 (PWM 50Hz)
 *
 * Timer 설정: prescaler=179, ARR=19999 (180MHz → 1tick=1μs, 주기=20ms)
 * Pulse 값(=CCR)을 μs 단위로 그대로 쓰면 됨.
 *
 *   1000 μs → -90도
 *   1500 μs → 0도 (중립)
 *   2000 μs → +90도
 *
 * MG996R 실측은 모듈마다 약간 다를 수 있어 SERVO_MIN/MAX_US 로 보정.
 */
#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ───────────────── 펄스 폭 한계 ─────────────────
 * 실측 결과에 따라 미세조정 가능.
 * MG996R 일반 사양: 1000~2000μs (180도 범위)
 * 안전마진: 900~2100 정도까지는 허용
 */
#define SERVO_MIN_US      1000   // -90도
#define SERVO_MID_US      1500   //   0도
#define SERVO_MAX_US      2000   // +90도

/* 짐벌 기계적 회전 한계 (소프트웨어 리미터)
 * 외측 프레임과 플레이트가 서로 충돌하지 않는 안전 각도
 * 처음 동작 시켜보고 좁히거나 넓히기
 */
#define GIMBAL_ANGLE_LIMIT_DEG  45.0f

typedef struct {
    TIM_HandleTypeDef *htim;   // 사용 타이머 핸들
    uint32_t channel;          // TIM_CHANNEL_1 또는 TIM_CHANNEL_2
    float    offset_deg;       // 중립 위치 오프셋 (기계적 캘리브레이션용)
    int      reverse;          // 1이면 부호 반전 (서보 방향이 반대일 때)
} Servo_t;

/**
 * @brief 서보 초기화 및 중립 위치로 이동
 */
void Servo_Init(Servo_t *s, TIM_HandleTypeDef *htim, uint32_t channel,
                float offset_deg, int reverse);

/**
 * @brief 각도(도)를 받아서 서보를 그 위치로 보냄
 *        내부에서 [-LIMIT, +LIMIT] 클램프 + offset/reverse 처리
 */
void Servo_WriteAngle(Servo_t *s, float angle_deg);

/**
 * @brief 모든 서보 PWM 출력 시작 (TIM_PWM_Start)
 *        Servo_Init 후 한번 호출
 */
void Servo_StartAll(TIM_HandleTypeDef *htim);

#endif
