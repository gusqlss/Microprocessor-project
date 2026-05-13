/**
 * @file    attitude.c
 * @brief   상보 필터 구현
 */
#include "attitude.h"
#include <math.h>

#define RAD_TO_DEG  (57.2957795f)

/* 가속도계로부터 Pitch/Roll 계산 (단위: 도)
 *
 * 좌표계 정의 (MPU6050 PCB 기준, 일반적 셋업):
 *   X축: 보드 길이 방향
 *   Y축: 보드 폭 방향
 *   Z축: 보드 위쪽 (중력 반대)
 *
 * Pitch = atan2(-Ax, sqrt(Ay² + Az²))   [X축이 들리면 +]
 * Roll  = atan2(Ay, Az)                 [Y축이 들리면 +]
 *
 * ⚠ 센서를 플레이트에 부착하는 방향에 따라 부호가 뒤집힐 수 있음.
 *   짐벌이 잘못된 방향으로 움직이면 부호만 바꿔주면 됨.
 */
static void accel_to_angles(MPU6050_t *dev, float *pitch_deg, float *roll_deg)
{
    float ax = dev->accel_g[0];
    float ay = dev->accel_g[1];
    float az = dev->accel_g[2];

    *pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    *roll_deg  = atan2f( ay, az)                       * RAD_TO_DEG;
}

/* ════════════════════════════════════════════════════════════
 * Attitude_Init
 * ════════════════════════════════════════════════════════════ */
void Attitude_Init(Attitude_t *att, MPU6050_t *dev, float alpha, float dt_sec)
{
    att->alpha = alpha;
    att->dt    = dt_sec;
    accel_to_angles(dev, &att->pitch, &att->roll);  // 가속도로 초기값 세팅
}

/* ════════════════════════════════════════════════════════════
 * Attitude_Update
 * ════════════════════════════════════════════════════════════ */
void Attitude_Update(Attitude_t *att, MPU6050_t *dev)
{
    /* 1) 가속도계로 즉시 추정한 Pitch/Roll */
    float pitch_acc, roll_acc;
    accel_to_angles(dev, &pitch_acc, &roll_acc);

    /* 2) 자이로 적분: 각속도(dps) × dt
     *
     *    Pitch 축: Y 자이로 (보드의 짧은 축 회전이 Pitch 변화)
     *    Roll  축: X 자이로
     *
     *  ⚠ 짐벌의 회전 방향과 자이로 부호가 안 맞으면 부호 뒤집기
     */
    float gyro_pitch = dev->gyro_dps[1];   // Y
    float gyro_roll  = dev->gyro_dps[0];   // X

    float pitch_gyro = att->pitch + gyro_pitch * att->dt;
    float roll_gyro  = att->roll  + gyro_roll  * att->dt;

    /* 3) 상보 필터로 융합 */
    att->pitch = att->alpha * pitch_gyro + (1.0f - att->alpha) * pitch_acc;
    att->roll  = att->alpha * roll_gyro  + (1.0f - att->alpha) * roll_acc;
}
