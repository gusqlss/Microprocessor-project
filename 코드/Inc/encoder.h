/**
 * @file    encoder.h
 * @brief   엔코더 EXTI 카운터 — 4채널
 *
 * JGB37-520 엔코더 2상 출력 (A상, B상) 중 A상만 EXTI 인터럽트로 받아 카운트.
 * 방향은 마지막 모터 PWM 명령 부호로 추정 (간이 방식).
 *
 * H/W Encoder Mode 안 쓰는 이유:
 *   - F446RE Nucleo (LQFP64)에서 TIM2/3/4/5 Encoder Mode 4개를
 *     동시에 핀 매핑하기 어려움 (PWM 핀과 겹침)
 *   - EXTI 4개면 핀 4개만 쓰고도 충분히 동작
 *
 * 정밀 PID가 필요하면 H/W Encoder Mode + B상까지 사용 권장.
 * 본 프로젝트는 시연용으로 펄스 카운트 + 누적 거리 추정 정도면 충분.
 *
 * 사용법 (CubeMX):
 *   1) 엔코더 A상 핀 4개를 GPIO_EXTI 로 설정 (Rising 또는 Both Edge)
 *   2) NVIC에서 해당 EXTI 라인 활성화
 *   3) main.c 에서 Encoder_OnInterrupt() 호출 (HAL_GPIO_EXTI_Callback)
 */
#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define ENC_COUNT  4

typedef struct {
    volatile int32_t count[ENC_COUNT];   // 누적 카운트 (음수 가능)
    int8_t           direction[ENC_COUNT]; // +1 또는 -1, main 코드가 SetDirection 으로 갱신
    uint16_t         exti_pin[ENC_COUNT];  // 어느 EXTI 핀인지 (콜백에서 매칭용)
} Encoder_t;

/** @brief 초기화 (카운트 0, 방향 +1) */
void Encoder_Init(Encoder_t *e,
                  uint16_t pin_fl, uint16_t pin_fr,
                  uint16_t pin_rl, uint16_t pin_rr);

/**
 * @brief 모터 PWM 부호에 따라 방향 갱신
 *        Mecanum_Drive 후 호출하는 게 좋음
 */
void Encoder_SetDirection(Encoder_t *e, int idx, float motor_speed);

/** @brief 카운트 읽기 (인터럽트 안전) */
int32_t Encoder_GetCount(Encoder_t *e, int idx);

/** @brief 모든 카운트 리셋 */
void Encoder_Reset(Encoder_t *e);

/**
 * @brief EXTI 콜백에서 호출 — GPIO_Pin 받아 어느 채널인지 매칭하고 카운트
 *        main.c 의 HAL_GPIO_EXTI_Callback() 안에서 호출
 */
void Encoder_OnInterrupt(Encoder_t *e, uint16_t GPIO_Pin);

#endif
