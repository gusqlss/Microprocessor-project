/**
 * @file    attitude.h
 * @brief   상보 필터 기반 자세 추정 (Pitch / Roll)
 *
 * - 가속도계: 정적일 때 정확한 각도, 동적일 때 노이즈 큼
 * - 자이로: 단기 응답 빠름, 장기 드리프트
 * → 상보 필터 (high-pass on gyro, low-pass on accel) 로 융합
 *
 *   angle = α * (angle + gyro * dt) + (1 - α) * accel_angle
 *
 *   α 는 보통 0.96 ~ 0.99 (gyro 가중치)
 *   짐벌처럼 정적 환경 위주면 0.96, 동적이면 0.98 ~ 0.99
 */
#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "mpu6050.h"

typedef struct {
    float pitch;        // 도 단위, 외측 프레임이 보정해야 할 축
    float roll;         // 도 단위, 플레이트가 보정해야 할 축
    float alpha;        // 상보 필터 계수 (0~1, gyro 가중치)
    float dt;           // 샘플 주기 (초)
} Attitude_t;

/**
 * @brief 자세 추정기 초기화
 *        가속도 값으로 초기 각도 세팅
 */
void Attitude_Init(Attitude_t *att, MPU6050_t *dev, float alpha, float dt_sec);

/**
 * @brief 한 스텝 업데이트
 *        루프 주기마다 호출 (예: 5ms = 200Hz)
 */
void Attitude_Update(Attitude_t *att, MPU6050_t *dev);

#endif
