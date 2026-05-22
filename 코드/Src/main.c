/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MPU6050 + 2축 짐벌 + 메카넘 4륜 통합본
  *                   (Day 5 + Day 8~11)
  ******************************************************************************
  * 동작 모드 (UART '1'~'3'):
  *   '1' = 짐벌만
  *   '2' = 수동 차량 (wasdqe) + 짐벌
  *   '3' = 라인 추종 + 짐벌 (라인센서 추가 후)
  *   '0' = 전체 정지
  *
  * 핀 매핑:
  *   짐벌 서보:  PA8(M1 Pitch), PA9(M2 Roll)        - TIM1
  *   MPU6050:    PB8(SCL), PB9(SDA)                 - I2C1
  *
  *   메카넘 RPWM (PWM):
  *     FL: PB6 (TIM4_CH1)
  *     FR: PC7 (TIM3_CH2)
  *     RL: PB10 (TIM2_CH3)
  *     RR: PB4 (TIM3_CH1)
  *
  *   메카넘 LPWM (GPIO Output):
  *     FL: PB5,  FR: PB3,  RL: PA10,  RR: PA7
  *
  *   디버그 UART: PA2(TX), PA3(RX)                  - USART2
  *   상태 LED:    PA5                                - LD2
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "mpu6050.h"
#include "servo.h"
#include "attitude.h"
#include "pid.h"
#include "bts7960.h"
#include "mecanum.h"

/* ───────── Handles ───────── */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef  htim1;   // 짐벌 서보 PWM
TIM_HandleTypeDef  htim2;   // 모터 RL RPWM (PB10)
TIM_HandleTypeDef  htim3;   // 모터 FR (PC7), RR (PB4) RPWM
TIM_HandleTypeDef  htim4;   // 모터 FL RPWM (PB6)
TIM_HandleTypeDef  htim6;   // 차량 100Hz 인터럽트
TIM_HandleTypeDef  htim7;   // 짐벌 200Hz 인터럽트

/* ───────── Global Structures ───────── */
MPU6050_t   mpu;
Servo_t     servo_pitch, servo_roll;
Attitude_t  hatti;
PID_t       pid_pitch, pid_roll;

BTS7960_t   wheel_fl, wheel_fr, wheel_rl, wheel_rr;
Mecanum_t   mecanum;

/* ───────── 모드 상태 ───────── */
typedef enum {
    MODE_IDLE = 0,         // 모든 동작 정지
    MODE_GIMBAL_ONLY,      // 짐벌만 동작
    MODE_MANUAL_DRIVE,     // 짐벌 + 수동 차량
    MODE_LINE_FOLLOW,      // 짐벌 + 라인 추종 (라인센서 추가 후)
} SystemMode_t;

volatile SystemMode_t system_mode = MODE_GIMBAL_ONLY;

/* UART 1바이트 수신 */
volatile uint8_t uart_rx_byte = 0;
volatile uint8_t uart_cmd_ready = 0;

/* 디버그 출력 카운터 */
volatile uint32_t debug_cnt = 0;

/* ───────── Prototypes ───────── */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
void MX_I2C1_Init(void);
void MX_TIM1_PWM_Init(void);
void MX_TIM7_Init(void);
void MX_MOTOR_PWM_Init(void);    // ★ 신규 - 모터 RPWM 4채널
void MX_MOTOR_GPIO_Init(void);   // ★ 신규 - 모터 LPWM 4핀
void MX_TIM6_Init(void);         // ★ 신규 - 차량 인터럽트
void Error_Handler(void);
static void Process_UartCommand(void);

/* printf → UART2 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* UART 수신 콜백 (1바이트 받으면 플래그 set) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uart_cmd_ready = 1;
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_byte, 1);
    }
}

/* ════════════════════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM1_PWM_Init();
    MX_TIM7_Init();

    /* ★★★ 메카넘 추가 ★★★ */
    MX_MOTOR_PWM_Init();
    MX_MOTOR_GPIO_Init();
    MX_TIM6_Init();

    HAL_Delay(1000);

    /* 부팅 로고 */
    printf("\r\n");
    printf("  #####   #####  #     #  ######     #     #      \r\n");
    printf(" #     #    #    ##   ##  #     #   # #    #      \r\n");
    printf(" #          #    # # # #  #     #  #   #   #      \r\n");
    printf(" #  ####    #    #  #  #  ######  #     #  #      \r\n");
    printf(" #     #    #    #     #  #     # #######  #      \r\n");
    printf("  #####   #####  #     #  ######  #     #  #####  \r\n");
    printf("\r\n");
    printf("====================================================\r\n");
    printf("  2-Axis Gimbal + Mecanum 4WD System Boot           \r\n");
    printf("====================================================\r\n\r\n");

    /* 1) MPU6050 */
    printf("1. Initializing MPU6050...\r\n");
    if (MPU6050_Init(&hi2c1) != HAL_OK) {
        printf("   - [FAIL] MPU6050 Init Error!\r\n");
        while (1) { HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); HAL_Delay(100); }
    }
    printf("   - [OK] MPU6050 Ready!\r\n");

    /* 2) 서보 2개 */
    printf("2. Initializing 2-Axis Servos...\r\n");
    Servo_Init(&servo_pitch, &htim1, TIM_CHANNEL_1, 0.0f, 0);
    Servo_Init(&servo_roll,  &htim1, TIM_CHANNEL_2, 0.0f, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_MOE_ENABLE(&htim1);
    printf("   - [OK] Servos Ready\r\n");

    /* 3) PID */
    printf("3. Initializing PID...\r\n");
    PID_Init(&pid_pitch, 1.2f, 0.05f, 0.1f, -45.0f, 45.0f, 20.0f, 0.005f);
    PID_Init(&pid_roll,  1.2f, 0.05f, 0.1f, -45.0f, 45.0f, 20.0f, 0.005f);
    printf("   - [OK] PID Ready\r\n");

    /* ★ 4) 메카넘 4륜 ★ */
    printf("4. Initializing Mecanum 4WD...\r\n");
    /* FL: RPWM=TIM4_CH1(PB6), LPWM=GPIO PB5 */
    BTS7960_Init(&wheel_fl, &htim4, TIM_CHANNEL_1, GPIOB, GPIO_PIN_5,  8999, 0);
    /* FR: RPWM=TIM3_CH2(PC7), LPWM=GPIO PB3 */
    BTS7960_Init(&wheel_fr, &htim3, TIM_CHANNEL_2, GPIOB, GPIO_PIN_3,  8999, 0);
    /* RL: RPWM=TIM2_CH3(PB10), LPWM=GPIO PA10 */
    BTS7960_Init(&wheel_rl, &htim2, TIM_CHANNEL_3, GPIOA, GPIO_PIN_10, 8999, 0);
    /* RR: RPWM=TIM3_CH1(PB4), LPWM=GPIO PA7 */
    BTS7960_Init(&wheel_rr, &htim3, TIM_CHANNEL_1, GPIOA, GPIO_PIN_7,  8999, 0);

    Mecanum_Init(&mecanum, &wheel_fl, &wheel_fr, &wheel_rl, &wheel_rr, 0.4f);
    printf("   - [OK] Mecanum Ready (LPWM=GPIO)\r\n");

    /* 5) UART RX 인터럽트 시작 + 짐벌 TIM7 시작 + 차량 TIM6 시작 */
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_byte, 1);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim6);

    printf("\r\n=== Ready. Commands ===\r\n");
    printf("  '1' = Gimbal only\r\n");
    printf("  '2' = Manual drive + Gimbal\r\n");
    printf("  '0' = STOP all\r\n");
    printf("  w/a/s/d = forward/strafeL/back/strafeR\r\n");
    printf("  q/e = turn L/R   space = stop wheels\r\n");
    printf("  t = FL motor test (정/역방향 2초씩)\r\n");
    printf("========================\r\n\r\n");

    /* main 루프: UART 명령 처리 + 상태 모니터 */
    while (1)
    {
        if (uart_cmd_ready) {
            uart_cmd_ready = 0;
            Process_UartCommand();
        }

        /* 1초마다 상태 출력 */
        if (++debug_cnt >= 200) {  // 5ms 인터럽트 × 200 = 1초
            debug_cnt = 0;
            const char *mode_name[] = { "IDLE", "GIMB", "MAN ", "LINE" };
            printf("[%s] P:%+6.2f R:%+6.2f\r\n",
                   mode_name[system_mode], hatti.pitch, hatti.roll);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }

        HAL_Delay(5);  // 200Hz polling
    }
}

/* ════════════════════════════════════════════════════════════
 * UART 1바이트 명령 처리
 * ════════════════════════════════════════════════════════════ */
static void Process_UartCommand(void)
{
    uint8_t c = uart_rx_byte;

    /* 모드 전환 */
    switch (c) {
        case '0':
            system_mode = MODE_IDLE;
            Mecanum_Stop(&mecanum);
            printf(">> MODE: IDLE\r\n");
            return;
        case '1':
            system_mode = MODE_GIMBAL_ONLY;
            Mecanum_Stop(&mecanum);
            printf(">> MODE: Gimbal only\r\n");
            return;
        case '2':
            system_mode = MODE_MANUAL_DRIVE;
            printf(">> MODE: Manual drive\r\n");
            return;

        /* 디버그 - FL 모터 단독 테스트 */
        case 't':
            printf("FL motor test: FWD\r\n");
            BTS7960_SetSpeed(&wheel_fl, 0.3f);
            HAL_Delay(2000);
            BTS7960_Stop(&wheel_fl);
            HAL_Delay(500);
            printf("FL motor test: REV\r\n");
            BTS7960_SetSpeed(&wheel_fl, -0.3f);
            HAL_Delay(2000);
            BTS7960_Stop(&wheel_fl);
            printf("FL test done\r\n");
            return;
    }

    /* MODE_MANUAL_DRIVE 일 때만 wasdqe 동작 */
    if (system_mode != MODE_MANUAL_DRIVE) return;

    switch (c) {
        case 'w': Mecanum_MoveForward (&mecanum, 0.4f); printf("> FWD\r\n");        break;
        case 's': Mecanum_MoveBackward(&mecanum, 0.4f); printf("> BWD\r\n");        break;
        case 'a': Mecanum_StrafeLeft  (&mecanum, 0.4f); printf("> LEFT strafe\r\n");break;
        case 'd': Mecanum_StrafeRight (&mecanum, 0.4f); printf("> RIGHT strafe\r\n"); break;
        case 'q': Mecanum_TurnLeft    (&mecanum, 0.4f); printf("> TURN L\r\n");     break;
        case 'e': Mecanum_TurnRight   (&mecanum, 0.4f); printf("> TURN R\r\n");     break;
        case ' ': Mecanum_Stop(&mecanum);               printf("> STOP\r\n");       break;
    }
}

/* ════════════════════════════════════════════════════════════
 * TIM7 인터럽트 (200Hz) - 짐벌 제어
 * ════════════════════════════════════════════════════════════ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        /* 짐벌은 IDLE 외에서 항상 동작 */
        if (system_mode == MODE_IDLE) {
            Servo_WriteAngle(&servo_pitch, 0.0f);
            Servo_WriteAngle(&servo_roll,  0.0f);
            return;
        }

        if (MPU6050_ReadAll(&hi2c1, &mpu) == HAL_OK) {
            Attitude_Update(&hatti, &mpu);

            float target = 0.0f;
            float u_pitch = PID_Compute(&pid_pitch, target, hatti.pitch);
            float u_roll  = PID_Compute(&pid_roll,  target, hatti.roll);

            Servo_WriteAngle(&servo_pitch, u_pitch);
            Servo_WriteAngle(&servo_roll,  u_roll);
        }
    }
    else if (htim->Instance == TIM6)
    {
        /* TIM6 (차량 100Hz) - 지금은 비워둠. 라인 추종 추가 시 여기에 LineFollow_Update */
        /* 추가 모드별 처리는 main 루프에서 명령으로 처리 중 */
    }
}

/* ════════════════════════════════════════════════════════════
 * TIM1 (PA8/PA9) 서보 PWM 50Hz - 기존 유지
 * ════════════════════════════════════════════════════════════ */
void MX_TIM1_PWM_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 83;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 19999;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = 1500;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

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
 * ★ NEW: 모터 RPWM 4채널 (TIM2_CH3, TIM3_CH1, TIM3_CH2, TIM4_CH1)
 * PWM 약 9.3kHz @ 84MHz
 * ════════════════════════════════════════════════════════════ */
void MX_MOTOR_PWM_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    /* PB6 = TIM4_CH1 (FL RPWM) */
    g.Pin = GPIO_PIN_6;  g.Alternate = GPIO_AF2_TIM4;  HAL_GPIO_Init(GPIOB, &g);
    /* PB4 = TIM3_CH1 (RR RPWM) */
    g.Pin = GPIO_PIN_4;  g.Alternate = GPIO_AF2_TIM3;  HAL_GPIO_Init(GPIOB, &g);
    /* PB10 = TIM2_CH3 (RL RPWM) */
    g.Pin = GPIO_PIN_10; g.Alternate = GPIO_AF1_TIM2;  HAL_GPIO_Init(GPIOB, &g);
    /* PC7 = TIM3_CH2 (FR RPWM) */
    g.Pin = GPIO_PIN_7;  g.Alternate = GPIO_AF2_TIM3;  HAL_GPIO_Init(GPIOC, &g);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    /* TIM2 (RL) - CH3 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 8999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_3);

    /* TIM3 (FR, RR) - CH1, CH2 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 8999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2);

    /* TIM4 (FL) - CH1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 8999;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();
    HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_1);
}

/* ════════════════════════════════════════════════════════════
 * ★ NEW: 모터 LPWM 4핀 GPIO Output (PB5, PB3, PA10, PA7)
 * ════════════════════════════════════════════════════════════ */
void MX_MOTOR_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    /* PB5 (FL LPWM), PB3 (FR LPWM) */
    g.Pin = GPIO_PIN_5 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &g);

    /* PA10 (RL LPWM), PA7 (RR LPWM) */
    g.Pin = GPIO_PIN_10 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &g);

    /* 모두 LOW로 초기화 (정지) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}

/* ════════════════════════════════════════════════════════════
 * ★ NEW: TIM6 차량 인터럽트 (100Hz)
 * ════════════════════════════════════════════════════════════ */
void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 83;       // 84MHz / 84 = 1MHz
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 9999;         // 10ms = 100Hz
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* ════════════════════════════════════════════════════════════
 * 시스템 클럭 - 기존 84MHz 유지
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

    /* ★★★ USART2 인터럽트 활성화 추가 ★★★ */
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
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

void MX_TIM7_Init(void)
{
    __HAL_RCC_TIM7_CLK_ENABLE();

    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 83;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 4999;
    htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);   /* 최우선 */
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
