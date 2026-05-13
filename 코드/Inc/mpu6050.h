/**
 * @file    mpu6050.h
 * @brief   MPU6050 (GY-521) 드라이버 — STM32 HAL 기반
 * @author  Gimbal Project
 *
 * I2C로 MPU6050에 접근하여 가속도/자이로 raw 데이터를 읽고,
 * 물리 단위(g, dps)로 변환한다.
 * 자세 추정(상보 필터)은 attitude.c 에서 별도로 수행한다.
 */
#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ───────────────── MPU6050 I2C 주소 ───────────────── */
/* AD0 = GND → 7-bit addr 0x68, HAL은 8-bit 받으므로 << 1 */
#define MPU6050_ADDR        (0x68 << 1)

/* ───────────────── 레지스터 주소 ───────────────── */
#define MPU6050_REG_SMPLRT_DIV   0x19  // 샘플레이트 분주
#define MPU6050_REG_CONFIG       0x1A  // DLPF 설정
#define MPU6050_REG_GYRO_CONFIG  0x1B  // 자이로 풀스케일
#define MPU6050_REG_ACCEL_CONFIG 0x1C  // 가속도 풀스케일
#define MPU6050_REG_ACCEL_XOUT_H 0x3B  // 가속도/자이로 14바이트 burst 시작
#define MPU6050_REG_PWR_MGMT_1   0x6B  // 전원 관리 (sleep wake)
#define MPU6050_REG_WHO_AM_I     0x75  // ID 확인 (0x68 반환)

/* ───────────────── 변환 스케일 ─────────────────
 * 본 프로젝트 기본 설정:
 *   - 가속도 풀스케일: ±2g  → 16384 LSB/g
 *   - 자이로 풀스케일: ±500dps → 65.5 LSB/dps
 *   (서보 짐벌 회전 속도는 빠르지 않으므로 ±500dps면 충분)
 */
#define ACCEL_LSB_PER_G    16384.0f
#define GYRO_LSB_PER_DPS   65.5f

/* ───────────────── 데이터 구조체 ───────────────── */
typedef struct {
    /* Raw 값 (보정 전) */
    int16_t accel_raw[3];   // [0]=X, [1]=Y, [2]=Z
    int16_t gyro_raw[3];
    int16_t temp_raw;

    /* 변환된 SI 값 */
    float accel_g[3];       // 단위: g
    float gyro_dps[3];      // 단위: degree per second
    float temp_c;           // 섭씨

    /* 자이로 바이어스 (정지 상태 보정값) */
    float gyro_bias[3];
} MPU6050_t;

/* ───────────────── 함수 프로토타입 ───────────────── */

/**
 * @brief MPU6050 초기화
 * @return HAL_OK / HAL_ERROR
 *
 * - WHO_AM_I 확인
 * - sleep 해제 (PWR_MGMT_1 = 0)
 * - 샘플레이트 1kHz (SMPLRT_DIV = 0)
 * - DLPF Bandwidth ~44Hz (CONFIG = 3)
 * - 자이로 ±500dps (GYRO_CONFIG = 0x08)
 * - 가속도 ±2g (ACCEL_CONFIG = 0x00)
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief 가속도/자이로 raw 14바이트를 burst read 후 SI 단위로 변환
 *        gyro_bias 가 미리 calibrate 되어있다면 자동 감산
 */
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_t *dev);

/**
 * @brief 자이로 바이어스 calibration
 *        센서를 평평한 곳에 정지시킨 상태에서 호출
 *        500 샘플 평균을 bias 로 저장
 * @note  호출 동안 약 1~2초 동안 절대 움직이지 말 것
 */
HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, MPU6050_t *dev);

#endif /* __MPU6050_H */
