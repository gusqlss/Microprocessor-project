/**
 * @file    pid.h
 * @brief   PID 제어기 (Anti-windup 포함)
 *
 * 짐벌은 setpoint = 0 (수평) 고정.
 * 제어 입력 u = 서보 각도 명령 (도)
 *
 *   error = setpoint - measured
 *   u = Kp*e + Ki*∫e dt + Kd*de/dt
 *
 * Anti-windup: 적분항이 출력 한계를 넘으면 적분 누적 중단 (clamping)
 * 미분 noise 완화: D term은 측정값 기반 (derivative on measurement)
 */
#ifndef __PID_H
#define __PID_H

typedef struct {
    /* 게인 */
    float Kp;
    float Ki;
    float Kd;

    /* 내부 상태 */
    float integral;         // 적분 누적값
    float prev_measurement; // D term 용 (derivative on measurement)
    float prev_error;       // 디버깅/대안용

    /* 한계 */
    float out_min;          // 출력 하한 (도)
    float out_max;          // 출력 상한 (도)
    float integral_max;     // 적분 누적 한계 (anti-windup)

    /* 샘플 주기 */
    float dt;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max,
              float integral_max, float dt_sec);

/**
 * @brief PID 한 스텝 계산
 * @param setpoint    목표값 (도). 짐벌은 보통 0
 * @param measurement 현재 측정값 (도)
 * @return            제어 출력 (도) — out_min ~ out_max 로 클램프됨
 */
float PID_Compute(PID_t *pid, float setpoint, float measurement);

/**
 * @brief 내부 상태 초기화 (시동 시 큰 적분 튀는 것 방지)
 */
void PID_Reset(PID_t *pid);

#endif
