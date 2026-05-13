/**
 * @file    linefollow.h
 * @brief   라인 추종 PID — 센서 위치 → 회전 명령
 *
 * 목표: 라인이 항상 중앙(position = 0)에 오도록 회전 보정
 *
 *   error = 0 - position    (라인이 오른쪽에 있으면 error < 0 → 우회전)
 *   ω = Kp*e + Ki*∫e + Kd*de/dt
 *
 * 본 시스템은 일반 PID 구조체 그대로 활용 가능하지만,
 * 라인트레이서 특성에 맞춘 thin wrapper 모듈 제공.
 *
 * 시연용 기본 동작:
 *   - 라인 검출 정상 → 전진 + PID로 회전 보정
 *   - 라인 잃음     → 정지 (또는 마지막 회전 방향으로 짧게 탐색)
 */
#ifndef __LINEFOLLOW_H
#define __LINEFOLLOW_H

#include "linesensor.h"
#include "pid.h"
#include "mecanum.h"

typedef struct {
    LineSensor_t *sensor;
    Mecanum_t    *mecanum;
    PID_t         pid;          // 라인 위치 PID

    float         base_speed;   // 기본 전진 속도 (0.0~1.0)
    float         turn_gain;    // 회전 명령 스케일 (0.5~1.5 시작)
    bool          stop_on_lost; // 라인 잃으면 정지 여부
} LineFollow_t;

/**
 * @brief 초기화
 * @param base_speed  기본 전진 속도 (0.2~0.4 부터 시작 추천)
 * @param Kp,Ki,Kd    라인 PID 게인 (예: 0.6, 0.0, 0.1)
 * @param dt_sec      제어 주기 (예: 0.01 = 100Hz)
 */
void LineFollow_Init(LineFollow_t *lf,
                     LineSensor_t *sensor, Mecanum_t *mecanum,
                     float base_speed,
                     float Kp, float Ki, float Kd,
                     float dt_sec);

/**
 * @brief 한 스텝 — 센서 읽고 PID 돌리고 메카넘에 명령
 *        제어 루프마다 호출
 */
void LineFollow_Update(LineFollow_t *lf);

/** @brief 비활성화 (정지) */
void LineFollow_Stop(LineFollow_t *lf);

#endif
