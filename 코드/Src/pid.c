/**
 * @file    pid.c
 * @brief   PID 구현 — Anti-windup + Derivative on Measurement
 */
#include "pid.h"

/* ════════════════════════════════════════════════════════════ */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max,
              float integral_max, float dt_sec)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_max = integral_max;
    pid->dt = dt_sec;
    PID_Reset(pid);
}

/* ════════════════════════════════════════════════════════════ */
void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_error = 0.0f;
}

/* ════════════════════════════════════════════════════════════ */
float PID_Compute(PID_t *pid, float setpoint, float measurement)
{
    /* 1) 오차 계산 */
    float error = setpoint - measurement;

    /* 2) P term */
    float P_out = pid->Kp * error;

    /* 3) I term — Anti-windup (clamping)
     *    적분이 한계 넘으면 saturation, 출력이 한계 풀리면 자연 해제 */
    pid->integral += error * pid->dt;
    if      (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float I_out = pid->Ki * pid->integral;

    /* 4) D term — Derivative on Measurement (노이즈 완화)
     *    de/dt = -d(meas)/dt   (setpoint 가 갑자기 바뀌어도 D kick 없음) */
    float d_meas = (measurement - pid->prev_measurement) / pid->dt;
    float D_out  = -pid->Kd * d_meas;

    pid->prev_measurement = measurement;
    pid->prev_error = error;

    /* 5) 합산 + 출력 클램프 */
    float u = P_out + I_out + D_out;
    if      (u > pid->out_max) u = pid->out_max;
    else if (u < pid->out_min) u = pid->out_min;

    return u;
}
