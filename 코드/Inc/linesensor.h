/**
 * @file    linesensor.h
 * @brief   TCRT5000 5채널 라인 센서 — 라인 위치 계산
 *
 * 5개 센서 배치 (앞쪽에서 봤을 때):
 *
 *   S1   S2   S3   S4   S5
 *   ━━━━━━━━━━━━━━━━━━━━━━━
 *   왼←        중앙        →오른
 *
 * 각 센서: 검은선 위 = LOW (0), 흰바닥 = HIGH (1)
 *          (TCRT5000 보드 종류에 따라 반대일 수도 → ACTIVE_LOW 매크로로 조정)
 *
 * 라인 위치 추정 (Weighted Average):
 *   position = Σ(i × b_i) / Σ(b_i)
 *   여기서 b_i = i번째 센서가 검은선이면 1, 아니면 0
 *
 *   position 범위: -2.0 ~ +2.0
 *     -2 = 가장 왼쪽 (S1만 라인 검출)
 *      0 = 정중앙   (S3만 검출 또는 좌우 대칭)
 *     +2 = 가장 오른쪽 (S5만 검출)
 *
 * 모든 센서가 흰바닥이면 (라인 잃음) → 마지막 위치 유지 + lost 플래그 set
 */
#ifndef __LINESENSOR_H
#define __LINESENSOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define LINE_SENSOR_COUNT  5

/* 센서 출력 극성:
 *   ACTIVE_LOW = 1 → 검은선 위에서 LOW (TCRT5000 일반 모듈 표준)
 *   ACTIVE_LOW = 0 → 검은선 위에서 HIGH
 * 처음 켰을 때 부팅 메시지의 raw 값 보고 결정하면 됨
 */
#define LINE_ACTIVE_LOW    1

typedef struct {
    GPIO_TypeDef *port[LINE_SENSOR_COUNT];
    uint16_t      pin [LINE_SENSOR_COUNT];

    uint8_t  raw[LINE_SENSOR_COUNT];  // 0/1 비트 (전처리 후)
    float    position;                // -2.0 ~ +2.0
    bool     line_lost;               // 모든 센서가 흰바닥일 때 true
    float    last_known_position;     // lost 시 직전 값 유지
} LineSensor_t;

/**
 * @brief 5채널 라인 센서 초기화
 *        port/pin 5쌍을 각각 인자로 받음
 */
void LineSensor_Init(LineSensor_t *ls,
                     GPIO_TypeDef *p1, uint16_t pin1,
                     GPIO_TypeDef *p2, uint16_t pin2,
                     GPIO_TypeDef *p3, uint16_t pin3,
                     GPIO_TypeDef *p4, uint16_t pin4,
                     GPIO_TypeDef *p5, uint16_t pin5);

/**
 * @brief 5채널 읽고 라인 위치 계산
 *        제어 루프마다 호출
 */
void LineSensor_Read(LineSensor_t *ls);

#endif
