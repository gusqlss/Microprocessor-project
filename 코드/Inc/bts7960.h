/**
 * @file    bts7960.h
 * @brief   BTS7960 H-브리지 모터 드라이버 — 단일 모터 제어
 *
 * BTS7960 모듈 1개 = 모터 1개 양방향 제어
 *   - RPWM (우방향 PWM): 정방향 회전 시 PWM
 *   - LPWM (좌방향 PWM): 역방향 회전 시 PWM
 *   - R_EN, L_EN: enable. 이 프로젝트에서는 5V 직결로 항상 활성화
 *
 * 한쪽만 PWM, 다른 쪽은 0으로 고정 (둘 다 PWM이면 쇼트 위험)
 *   정회전:   RPWM = duty, LPWM = 0
 *   역회전:   RPWM = 0,    LPWM = duty
 *   브레이크: RPWM = 0,    LPWM = 0  (또는 둘 다 HIGH = 코스트)
 *
 * PWM 주파수: 1~20kHz 권장 (모터 소음 vs 효율 절충, 본 코드 20kHz)
 */
#ifndef __BTS7960_H
#define __BTS7960_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim_rpwm;
    uint32_t           ch_rpwm;
    TIM_HandleTypeDef *htim_lpwm;
    uint32_t           ch_lpwm;
    uint32_t           arr;     // 타이머 ARR (PWM 최댓값)
    int                reverse; // 1이면 회전 방향 반전
} BTS7960_t;

/**
 * @brief 모터 초기화 (PWM 시작 포함)
 *        Init 후 자동으로 정지 상태
 */
void BTS7960_Init(BTS7960_t *m,
                  TIM_HandleTypeDef *htim_rpwm, uint32_t ch_rpwm,
                  TIM_HandleTypeDef *htim_lpwm, uint32_t ch_lpwm,
                  uint32_t arr, int reverse);

/**
 * @brief 모터 속도 설정
 * @param speed  -1.0 ~ +1.0 (음수 역회전, 0 정지, +1.0 최대 정회전)
 *               범위 벗어나면 자동 클램프
 */
void BTS7960_SetSpeed(BTS7960_t *m, float speed);

/** @brief 즉시 정지 (소프트 브레이크) */
void BTS7960_Stop(BTS7960_t *m);

#endif
