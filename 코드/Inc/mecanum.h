/**
 * @file    mecanum.h
 * @brief   메카넘 4륜 운동학 (Inverse Kinematics)
 *
 * 3축 명령 (전진 vx, 횡이동 vy, 회전 ω) → 4륜 속도
 *
 *   FL = vx − vy − ω    (앞좌)
 *   FR = vx + vy + ω    (앞우)
 *   RL = vx + vy − ω    (뒤좌)
 *   RR = vx − vy + ω    (뒤우)
 *
 *   ※ L_x, L_y (휠베이스/트랙)는 본 단순화 버전에서 ω 에 묶어 처리
 *
 * 입력 범위: 각 축 -1.0 ~ +1.0
 * 출력 범위: 4륜 속도 합이 1.0 넘으면 자동 정규화 (saturation 방지)
 *
 * 좌표계 (위에서 봤을 때):
 *           ↑ +vx (전진)
 *           |
 *   FL  ━━━━━━━━ FR
 *    ┃    ↓+ω    ┃     ← +ω = 좌회전 (반시계, 위에서 본 기준)
 *    ┃→ +vy      ┃
 *   RL  ━━━━━━━━ RR
 *
 * ⚠ 메카넘 휠 롤러 방향:
 *   FL, RR : "\\" 방향 롤러
 *   FR, RL : "//" 방향 롤러
 *   위에서 봤을 때 4개가 X자 패턴. 잘못 조립하면 횡이동 안 됨.
 */
#ifndef __MECANUM_H
#define __MECANUM_H

#include "bts7960.h"

typedef enum {
    WHEEL_FL = 0,  // Front Left
    WHEEL_FR = 1,  // Front Right
    WHEEL_RL = 2,  // Rear  Left
    WHEEL_RR = 3,  // Rear  Right
    WHEEL_COUNT = 4
} WheelIndex_t;

typedef struct {
    BTS7960_t *wheels[WHEEL_COUNT];  // 4륜 모터 핸들 (포인터)
    float      max_output;           // 전체 최대 출력 한계 (0.0~1.0)
} Mecanum_t;

/**
 * @brief 메카넘 시스템 초기화
 * @param fl, fr, rl, rr  미리 BTS7960_Init 된 모터 핸들 4개
 * @param max_output      안전 최대치 (0.4~0.7 추천, 처음엔 작게 시작)
 */
void Mecanum_Init(Mecanum_t *m,
                  BTS7960_t *fl, BTS7960_t *fr,
                  BTS7960_t *rl, BTS7960_t *rr,
                  float max_output);

/**
 * @brief 3축 명령 → 4륜 속도 분배 + 모터 출력
 * @param vx  전진/후진  [-1.0, +1.0]   (+ 전진)
 * @param vy  좌/우 횡이동 [-1.0, +1.0]  (+ 오른쪽)
 * @param wz  회전 각속도  [-1.0, +1.0]  (+ 좌회전 = 반시계)
 *
 *   세 입력의 합 절댓값이 1.0 넘으면 비례 축소(정규화)
 */
void Mecanum_Drive(Mecanum_t *m, float vx, float vy, float wz);

/** @brief 즉시 4륜 정지 */
void Mecanum_Stop(Mecanum_t *m);

/* ───────────────── 편의 함수 (자주 쓸 동작 미리 정의) ───────────────── */
void Mecanum_MoveForward(Mecanum_t *m, float speed);   // 전진
void Mecanum_MoveBackward(Mecanum_t *m, float speed);  // 후진
void Mecanum_StrafeLeft(Mecanum_t *m, float speed);    // 왼쪽 평행이동 (요청한 기능!)
void Mecanum_StrafeRight(Mecanum_t *m, float speed);   // 오른쪽 평행이동
void Mecanum_TurnLeft(Mecanum_t *m, float speed);      // 제자리 좌회전
void Mecanum_TurnRight(Mecanum_t *m, float speed);     // 제자리 우회전

#endif
