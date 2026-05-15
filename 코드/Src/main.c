/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MPU6050 + Servo(M1, TIM1_CH1/PA8) — Day4 통합본
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include "mpu6050.h"
#include "servo.h"

/* ───────── Handles ───────── */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef  htim1;

MPU6050_t mpu;
Servo_t   servoM1;

/* ───────── Prototypes ───────── */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
void MX_I2C1_Init(void);
void MX_TIM1_PWM_Init(void);
void Error_Handler(void);

/* printf → UART2 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* ════════════════════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM1_PWM_Init();          /* ★ 서보 PWM */

    HAL_Delay(1000);
    printf("\r\n\n=== MPU6050 + SERVO(M1) START ===\r\n");
    printf("SYSCLK = %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
    printf("PCLK2  = %lu Hz (TIM1 clk = same when APB2 div=1)\r\n",
           HAL_RCC_GetPCLK2Freq());

    /* 1) I2C 스캔 */
    printf("1. Scanning I2C bus...\r\n");
    for (uint16_t i = 1; i < 128; i++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 10) == HAL_OK)
            printf("   - Found device at: 0x%02X\r\n", i);
    }

    /* 2) MPU6050 초기화 */
    printf("2. Initializing MPU6050...\r\n");
    if (MPU6050_Init(&hi2c1) != HAL_OK) {
        printf("   - [FAIL] MPU6050 Init Error!\r\n");
        while (1) { HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); HAL_Delay(100); }
    }
    printf("   - [OK] MPU6050 Ready!\r\n");

    /* 3) 서보 초기화 + PWM 시작 */
    printf("3. Initializing Servo M1 (PA8 / TIM1_CH1)...\r\n");
    Servo_Init(&servoM1, &htim1, TIM_CHANNEL_1, 0.0f, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_MOE_ENABLE(&htim1);       /* ★ TIM1은 advanced timer → MOE 필수 */
    HAL_Delay(500);
    printf("   - [OK] Neutral (0 deg)\r\n\n");

    /* 4) 스윕 시퀀스 */
    const float seq[] = { -30.0f, 0.0f, +30.0f, 0.0f };
    const int   N     = sizeof(seq) / sizeof(seq[0]);
    int idx = 0;

    while (1)
    {
        float target = seq[idx];
        Servo_WriteAngle(&servoM1, target);
        printf(">> Servo target = %+6.1f deg | CCR1 = %lu\r\n",
               target, (unsigned long)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1));

        for (int k = 0; k < 10; k++) {
            if (MPU6050_ReadAll(&hi2c1, &mpu) == HAL_OK) {
                printf("ACC: %+.2f, %+.2f, %+.2f | GYR: %+.2f, %+.2f, %+.2f\r\n",
                       mpu.accel_g[0], mpu.accel_g[1], mpu.accel_g[2],
                       mpu.gyro_dps[0], mpu.gyro_dps[1], mpu.gyro_dps[2]);
            }
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            HAL_Delay(100);
        }
        idx = (idx + 1) % N;
    }
}

/* ════════════════════════════════════════════════════════════
 * TIM1 CH1 PWM (50Hz)  — PA8
 * SYSCLK=84MHz, APB2=84MHz, TIM1 clk=84MHz
 *   Prescaler = 83   →  84MHz / 84 = 1 MHz  → 1 tick = 1 µs
 *   Period    = 19999 → 20,000 µs = 20 ms  → 50 Hz
 * ════════════════════════════════════════════════════════════ */
void MX_TIM1_PWM_Init(void)
{
    /* (a) 클럭 ON */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* (b) PA8 → AF1 (TIM1_CH1), Push-Pull */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_8;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* (c) 타이머 base */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 83;        /* ★ 84MHz/(83+1) = 1MHz */
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 19999;     /* 20ms */
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    /* (d) PWM 채널 설정 */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = 1500;                   /* 중립 1.5ms */
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    /* (e) advanced timer → Break/Dead-Time, MOE 자동 출력 */
    TIM_BreakDeadTimeConfigTypeDef bd = {0};
    bd.OffStateRunMode  = TIM_OSSR_DISABLE;
    bd.OffStateIDLEMode = TIM_OSSI_DISABLE;
    bd.LockLevel        = TIM_LOCKLEVEL_OFF;
    bd.DeadTime         = 0;
    bd.BreakState       = TIM_BREAK_DISABLE;
    bd.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    bd.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bd);
}

/* ════════════════════════════════════════════════════════════
 *  아래는 기존 코드 그대로
 * ════════════════════════════════════════════════════════════ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
