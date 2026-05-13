/**
 * @file    mecanum.c
 */
#include "mecanum.h"
#include <math.h>

static float abs_f(float x) { return x < 0 ? -x : x; }

void Mecanum_Init(Mecanum_t *m,
                  BTS7960_t *fl, BTS7960_t *fr,
                  BTS7960_t *rl, BTS7960_t *rr,
                  float max_output)
{
    m->wheels[WHEEL_FL] = fl;
    m->wheels[WHEEL_FR] = fr;
    m->wheels[WHEEL_RL] = rl;
    m->wheels[WHEEL_RR] = rr;

    if (max_output <= 0.0f) max_output = 0.5f;   // 안전 기본값
    if (max_output >  1.0f) max_output = 1.0f;
    m->max_output = max_output;

    Mecanum_Stop(m);
}

void Mecanum_Drive(Mecanum_t *m, float vx, float vy, float wz)
{
    /* 1) 입력 클램프 */
    if (vx >  1.0f) vx =  1.0f;  if (vx < -1.0f) vx = -1.0f;
    if (vy >  1.0f) vy =  1.0f;  if (vy < -1.0f) vy = -1.0f;
    if (wz >  1.0f) wz =  1.0f;  if (wz < -1.0f) wz = -1.0f;

    /* 2) 메카넘 역운동학
     *
     *   FL = vx − vy − ω
     *   FR = vx + vy + ω
     *   RL = vx + vy − ω
     *   RR = vx − vy + ω
     */
    float w[WHEEL_COUNT];
    w[WHEEL_FL] = vx - vy - wz;
    w[WHEEL_FR] = vx + vy + wz;
    w[WHEEL_RL] = vx + vy - wz;
    w[WHEEL_RR] = vx - vy + wz;

    /* 3) 정규화 — 4개 중 최대 절댓값이 1.0 넘으면 모두 비례 축소
     *    (saturation 시 한쪽만 잘리면 차체가 의도와 다르게 움직임) */
    float max_mag = abs_f(w[0]);
    for (int i = 1; i < WHEEL_COUNT; i++) {
        float a = abs_f(w[i]);
        if (a > max_mag) max_mag = a;
    }
    if (max_mag > 1.0f) {
        for (int i = 0; i < WHEEL_COUNT; i++) w[i] /= max_mag;
    }

    /* 4) 안전 max_output 곱하고 모터 출력 */
    for (int i = 0; i < WHEEL_COUNT; i++) {
        BTS7960_SetSpeed(m->wheels[i], w[i] * m->max_output);
    }
}

void Mecanum_Stop(Mecanum_t *m)
{
    for (int i = 0; i < WHEEL_COUNT; i++) {
        BTS7960_Stop(m->wheels[i]);
    }
}

/* ───────── 편의 동작 함수 ───────── */
void Mecanum_MoveForward (Mecanum_t *m, float s) { Mecanum_Drive(m,  s, 0,  0); }
void Mecanum_MoveBackward(Mecanum_t *m, float s) { Mecanum_Drive(m, -s, 0,  0); }
void Mecanum_StrafeLeft  (Mecanum_t *m, float s) { Mecanum_Drive(m,  0,-s,  0); }
void Mecanum_StrafeRight (Mecanum_t *m, float s) { Mecanum_Drive(m,  0, s,  0); }
void Mecanum_TurnLeft    (Mecanum_t *m, float s) { Mecanum_Drive(m,  0, 0,  s); }
void Mecanum_TurnRight   (Mecanum_t *m, float s) { Mecanum_Drive(m,  0, 0, -s); }
