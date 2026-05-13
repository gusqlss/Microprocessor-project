/**
 * @file    mpu6050.c
 * @brief   MPU6050 드라이버 구현
 */
#include "mpu6050.h"
#include <string.h>

#define I2C_TIMEOUT  100  // ms

/* 정적 헬퍼 함수: 1바이트 쓰기 */
static HAL_StatusTypeDef mpu_write_byte(I2C_HandleTypeDef *hi2c,
                                         uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, reg, 1, &val, 1, I2C_TIMEOUT);
}

/* 정적 헬퍼 함수: 다중 바이트 읽기 */
static HAL_StatusTypeDef mpu_read(I2C_HandleTypeDef *hi2c,
                                   uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, reg, 1, buf, len, I2C_TIMEOUT);
}

/* ════════════════════════════════════════════════════════════
 * MPU6050_Init
 * ════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0;
    HAL_StatusTypeDef st;

    /* 1) WHO_AM_I 확인 → 0x68 반환되어야 정상
     *    AD0 핀이 HIGH면 0x69 반환되니까 결선 확인 필수 */
    st = mpu_read(hi2c, MPU6050_REG_WHO_AM_I, &who, 1);
    if (st != HAL_OK) return st;
    if (who != 0x68) return HAL_ERROR;   // 센서 응답 이상

    /* 2) PWR_MGMT_1 = 0x00 → sleep 해제, 내부 8MHz 클럭 사용 */
    st = mpu_write_byte(hi2c, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (st != HAL_OK) return st;
    HAL_Delay(50);  // wake up 안정화

    /* 3) SMPLRT_DIV = 0 → sample rate = 1kHz (DLPF 활성화 가정시)
     *    실제로는 우리가 200Hz로 메인 루프에서 읽음 */
    st = mpu_write_byte(hi2c, MPU6050_REG_SMPLRT_DIV, 0x00);
    if (st != HAL_OK) return st;

    /* 4) CONFIG = 0x03 → DLPF Bandwidth ≈ 44Hz (acc), 42Hz (gyro)
     *    노이즈 감소 효과 큼. 너무 낮추면 응답 느려지고, 높이면 노이즈↑ */
    st = mpu_write_byte(hi2c, MPU6050_REG_CONFIG, 0x03);
    if (st != HAL_OK) return st;

    /* 5) GYRO_CONFIG = 0x08 → 자이로 풀스케일 ±500 dps */
    st = mpu_write_byte(hi2c, MPU6050_REG_GYRO_CONFIG, 0x08);
    if (st != HAL_OK) return st;

    /* 6) ACCEL_CONFIG = 0x00 → 가속도 풀스케일 ±2g */
    st = mpu_write_byte(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (st != HAL_OK) return st;

    return HAL_OK;
}

/* ════════════════════════════════════════════════════════════
 * MPU6050_ReadAll
 * ════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_t *dev)
{
    uint8_t buf[14];
    HAL_StatusTypeDef st;

    /* 14 바이트 burst read (가속도 6, 온도 2, 자이로 6) */
    st = mpu_read(hi2c, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (st != HAL_OK) return st;

    /* Big-endian 정렬: H 바이트가 먼저 */
    dev->accel_raw[0] = (int16_t)((buf[0]  << 8) | buf[1]);
    dev->accel_raw[1] = (int16_t)((buf[2]  << 8) | buf[3]);
    dev->accel_raw[2] = (int16_t)((buf[4]  << 8) | buf[5]);
    dev->temp_raw     = (int16_t)((buf[6]  << 8) | buf[7]);
    dev->gyro_raw[0]  = (int16_t)((buf[8]  << 8) | buf[9]);
    dev->gyro_raw[1]  = (int16_t)((buf[10] << 8) | buf[11]);
    dev->gyro_raw[2]  = (int16_t)((buf[12] << 8) | buf[13]);

    /* SI 단위로 변환 */
    for (int i = 0; i < 3; i++) {
        dev->accel_g[i]   = (float)dev->accel_raw[i] / ACCEL_LSB_PER_G;
        /* 자이로는 바이어스 빼고 dps로 변환 */
        dev->gyro_dps[i]  = ((float)dev->gyro_raw[i] / GYRO_LSB_PER_DPS)
                            - dev->gyro_bias[i];
    }
    dev->temp_c = (float)dev->temp_raw / 340.0f + 36.53f;  // 데이터시트 공식

    return HAL_OK;
}

/* ════════════════════════════════════════════════════════════
 * MPU6050_CalibrateGyro
 * 정지 상태에서 500 샘플 평균 → bias 저장
 * ════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, MPU6050_t *dev)
{
    const int N = 500;
    float sum[3] = {0.0f, 0.0f, 0.0f};

    /* bias 0으로 초기화 후 raw 측정 */
    dev->gyro_bias[0] = 0.0f;
    dev->gyro_bias[1] = 0.0f;
    dev->gyro_bias[2] = 0.0f;

    for (int n = 0; n < N; n++) {
        if (MPU6050_ReadAll(hi2c, dev) != HAL_OK) return HAL_ERROR;
        sum[0] += dev->gyro_dps[0];
        sum[1] += dev->gyro_dps[1];
        sum[2] += dev->gyro_dps[2];
        HAL_Delay(2);  // ≈ 500 Hz 샘플링
    }

    dev->gyro_bias[0] = sum[0] / N;
    dev->gyro_bias[1] = sum[1] / N;
    dev->gyro_bias[2] = sum[2] / N;

    return HAL_OK;
}
