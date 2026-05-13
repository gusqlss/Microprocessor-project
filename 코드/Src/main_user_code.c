/* ╔═══════════════════════════════════════════════════════════════╗
 * ║   STM32 Nucleo-F446RE                                          ║
 * ║   2축 짐벌 + 메카넘 4륜 + 라인 트레이서 통합                    ║
 * ║                                                                ║
 * ║   ⚠ 이 파일은 CubeMX가 생성한 main.c의 USER CODE 영역에        ║
 * ║      복붙하기 위한 가이드입니다. 별도 파일 아님!               ║
 * ║                                                                ║
 * ║   동작:                                                         ║
 * ║     TIM7  (200Hz, 5ms) → 짐벌 제어 (IMU + PID + 서보)         ║
 * ║     TIM6  (100Hz, 10ms) → 차량 제어 (라인 + 메카넘)            ║
 * ║     EXTI         → 엔코더 펄스 카운트                           ║
 * ╚═══════════════════════════════════════════════════════════════╝
 *
 * 사용 타이머/주변장치 정리 (CubeMX에서 모두 설정 필요):
 *
 *   I2C1         — MPU6050         (PB8, PB9)        Fast 400kHz
 *   TIM1 CH1/2   — 짐벌 서보 PWM   (PA8, PA9)        50Hz
 *   TIM8 CH1-4   — BTS7960 RPWM    (PC6,PC7,PC8,PC9) 20kHz
 *   TIM4 CH1/2 + TIM5 CH3/4 — BTS7960 LPWM            20kHz
 *   TIM6         — 차량 100Hz 인터럽트 (NVIC 활성화)
 *   TIM7         — 짐벌 200Hz 인터럽트 (NVIC 활성화)
 *   EXTI         — 엔코더 4채널 GPIO
 *   USART2       — 디버그 (PA2, PA3)
 *   GPIO 입력 5  — 라인 센서 (PB0~PB4 등)
 */

/* ═══════════════════════════════════════════════════════════════
 * USER CODE BEGIN Includes
 * ═══════════════════════════════════════════════════════════════ */
#include "mpu6050.h"
#include "attitude.h"
#include "pid.h"
#include "servo.h"
#include "bts7960.h"
#include "mecanum.h"
#include "linesensor.h"
#include "linefollow.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */


/* ═══════════════════════════════════════════════════════════════
 * USER CODE BEGIN PD
 * ═══════════════════════════════════════════════════════════════ */

/* ───────── 제어 주기 ───────── */
#define GIMBAL_DT_SEC   0.005f    // 200 Hz (TIM7)
#define VEHICLE_DT_SEC  0.010f    // 100 Hz (TIM6)

/* ───────── 짐벌 PID 게인 (튜닝 시작점) ───────── */
#define PID_PITCH_KP    4.0f
#define PID_PITCH_KI    0.10f
#define PID_PITCH_KD    0.50f
#define PID_ROLL_KP     4.0f
#define PID_ROLL_KI     0.10f
#define PID_ROLL_KD     0.50f
#define PID_OUT_LIMIT   45.0f
#define PID_INT_LIMIT   20.0f

#define COMP_FILTER_ALPHA   0.98f

/* ───────── 서보 보정 (조립 후 실측) ───────── */
#define SERVO_PITCH_OFFSET   0.0f
#define SERVO_ROLL_OFFSET    0.0f
#define SERVO_PITCH_REVERSE  0
#define SERVO_ROLL_REVERSE   0

/* ───────── 메카넘 PWM 타이머 ARR ─────────
 * 20kHz PWM → 180MHz / (PSC+1) / (ARR+1) = 20000
 *   PSC = 0, ARR = 8999 → 분해능 9000 단계
 */
#define MOTOR_PWM_ARR   8999

/* ───────── 메카넘 안전 최대 출력 ─────────
 * 처음에 작게! 큰 차체가 갑자기 튀어나가면 위험.
 */
#define MECANUM_MAX_OUT   0.4f

/* ───────── 라인 추종 PID ───────── */
#define LINE_BASE_SPEED   0.3f
#define LINE_KP           0.6f
#define LINE_KI           0.0f
#define LINE_KD           0.10f

/* ───────── 모터 방향 반전 (조립 후 단독 테스트로 결정) ───────── */
#define WHEEL_FL_REVERSE   0
#define WHEEL_FR_REVERSE   0
#define WHEEL_RL_REVERSE   0
#define WHEEL_RR_REVERSE   0

#define DEBUG_PRINT_DECIM   50   // 100Hz * 50 = 0.5s

/* ───────── 시스템 모드 ───────── */
typedef enum {
    MODE_IDLE = 0,           // 모든 동작 정지
    MODE_GIMBAL_ONLY,        // 짐벌만 동작 (튜닝/테스트)
    MODE_MANUAL_DRIVE,       // UART 명령으로 차량 + 짐벌
    MODE_LINE_FOLLOW,        // 라인 추종 + 짐벌  ← 발표 시연
} SystemMode_t;

/* USER CODE END PD */


/* ═══════════════════════════════════════════════════════════════
 * USER CODE BEGIN PV
 * ═══════════════════════════════════════════════════════════════ */
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim1;   // 짐벌 서보 PWM
extern TIM_HandleTypeDef  htim4;   // 메카넘 LPWM
extern TIM_HandleTypeDef  htim5;   // 메카넘 LPWM
extern TIM_HandleTypeDef  htim6;   // 차량 100Hz 인터럽트
extern TIM_HandleTypeDef  htim7;   // 짐벌 200Hz 인터럽트
extern TIM_HandleTypeDef  htim8;   // 메카넘 RPWM
extern UART_HandleTypeDef huart2;

/* 짐벌 */
MPU6050_t   mpu;
Attitude_t  attitude;
PID_t       pid_pitch, pid_roll;
Servo_t     servo_pitch, servo_roll;

/* 메카넘 */
BTS7960_t   wheel_fl, wheel_fr, wheel_rl, wheel_rr;
Mecanum_t   mecanum;

/* 라인 */
LineSensor_t  line_sensor;
LineFollow_t  line_follow;

/* 엔코더 */
Encoder_t   encoder;

/* 시스템 상태 */
volatile SystemMode_t system_mode = MODE_IDLE;
volatile uint8_t gimbal_flag  = 0;
volatile uint8_t vehicle_flag = 0;
volatile uint32_t debug_cnt = 0;

volatile uint8_t uart_rx_byte = 0;
volatile uint8_t uart_cmd_ready = 0;

/* USER CODE END PV */


/* ═══════════════════════════════════════════════════════════════
 * USER CODE BEGIN PFP
 * ═══════════════════════════════════════════════════════════════ */
static void System_Init(void);
static void Gimbal_ControlLoop(void);
static void Vehicle_ControlLoop(void);
static void Debug_Print(void);
static void Error_Trap(const char *msg);
static void Process_UartCommand(void);
/* USER CODE END PFP */


/* ═══════════════════════════════════════════════════════════════
 * USER CODE BEGIN 0  — printf retarget + UART RX 콜백
 * ═══════════════════════════════════════════════════════════════ */
#ifdef __GNUC__
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
    return ch;
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uart_cmd_ready = 1;
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_byte, 1);
    }
}
/* USER CODE END 0 */


/* ═══════════════════════════════════════════════════════════════
 * USER CODE BEGIN 2  — HAL_Init 후, while(1) 직전
 * ═══════════════════════════════════════════════════════════════ */
System_Init();

HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_byte, 1);

HAL_TIM_Base_Start_IT(&htim7);   // 짐벌 200Hz
HAL_TIM_Base_Start_IT(&htim6);   // 차량 100Hz

printf("\r\nReady. Commands:\r\n");
printf("  '1' = Gimbal only\r\n");
printf("  '2' = Manual drive + Gimbal\r\n");
printf("  '3' = Line follow + Gimbal  (DEMO)\r\n");
printf("  '0' = STOP all\r\n");
printf("  w/a/s/d/q/e = forward/strafeL/back/strafeR/turnL/turnR (mode 2)\r\n");
printf("  space       = stop wheels\r\n");

system_mode = MODE_GIMBAL_ONLY;

/* USER CODE END 2 */


/* ═══════════════════════════════════════════════════════════════
 * USER CODE BEGIN WHILE  — main loop
 * ═══════════════════════════════════════════════════════════════ */
while (1)
{
    if (uart_cmd_ready) {
        uart_cmd_ready = 0;
        Process_UartCommand();
    }

    if (gimbal_flag) {
        gimbal_flag = 0;
        Gimbal_ControlLoop();
    }

    if (vehicle_flag) {
        vehicle_flag = 0;
        Vehicle_ControlLoop();

        if (++debug_cnt >= DEBUG_PRINT_DECIM) {
            debug_cnt = 0;
            Debug_Print();
        }
    }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */


/* ═══════════════════════════════════════════════════════════════
 * USER CODE BEGIN 4
 * ═══════════════════════════════════════════════════════════════ */

static void System_Init(void)
{
    printf("\r\n========================================\r\n");
    printf("  Gimbal + Mecanum Line Tracer Boot\r\n");
    printf("========================================\r\n");

    /* 짐벌 */
    if (MPU6050_Init(&hi2c1) != HAL_OK) Error_Trap("MPU6050 init");
    printf("[OK] MPU6050\r\n");

    printf("Calibrating gyro... DO NOT MOVE!\r\n");
    HAL_Delay(800);
    if (MPU6050_CalibrateGyro(&hi2c1, &mpu) != HAL_OK) Error_Trap("Gyro cal");
    printf("[OK] Gyro bias\r\n");

    MPU6050_ReadAll(&hi2c1, &mpu);
    Attitude_Init(&attitude, &mpu, COMP_FILTER_ALPHA, GIMBAL_DT_SEC);

    PID_Init(&pid_pitch, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
             -PID_OUT_LIMIT, PID_OUT_LIMIT, PID_INT_LIMIT, GIMBAL_DT_SEC);
    PID_Init(&pid_roll,  PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,
             -PID_OUT_LIMIT, PID_OUT_LIMIT, PID_INT_LIMIT, GIMBAL_DT_SEC);

    Servo_Init(&servo_pitch, &htim1, TIM_CHANNEL_1,
               SERVO_PITCH_OFFSET, SERVO_PITCH_REVERSE);
    Servo_Init(&servo_roll,  &htim1, TIM_CHANNEL_2,
               SERVO_ROLL_OFFSET,  SERVO_ROLL_REVERSE);
    Servo_StartAll(&htim1);
    Servo_WriteAngle(&servo_pitch, 0.0f);
    Servo_WriteAngle(&servo_roll,  0.0f);
    printf("[OK] Gimbal\r\n");

    /* 메카넘 4륜 — 핀은 CubeMX와 일치시킬 것 */
    BTS7960_Init(&wheel_fl, &htim8, TIM_CHANNEL_1, &htim4, TIM_CHANNEL_1,
                 MOTOR_PWM_ARR, WHEEL_FL_REVERSE);
    BTS7960_Init(&wheel_fr, &htim8, TIM_CHANNEL_2, &htim4, TIM_CHANNEL_2,
                 MOTOR_PWM_ARR, WHEEL_FR_REVERSE);
    BTS7960_Init(&wheel_rl, &htim8, TIM_CHANNEL_3, &htim5, TIM_CHANNEL_3,
                 MOTOR_PWM_ARR, WHEEL_RL_REVERSE);
    BTS7960_Init(&wheel_rr, &htim8, TIM_CHANNEL_4, &htim5, TIM_CHANNEL_4,
                 MOTOR_PWM_ARR, WHEEL_RR_REVERSE);

    Mecanum_Init(&mecanum, &wheel_fl, &wheel_fr, &wheel_rl, &wheel_rr,
                 MECANUM_MAX_OUT);
    printf("[OK] Mecanum\r\n");

    /* 라인 센서 — 핀 5개는 CubeMX에서 GPIO_Input 설정 */
    LineSensor_Init(&line_sensor,
                    GPIOB, GPIO_PIN_0,
                    GPIOB, GPIO_PIN_1,
                    GPIOB, GPIO_PIN_2,
                    GPIOB, GPIO_PIN_3,
                    GPIOB, GPIO_PIN_4);

    LineFollow_Init(&line_follow, &line_sensor, &mecanum,
                    LINE_BASE_SPEED, LINE_KP, LINE_KI, LINE_KD,
                    VEHICLE_DT_SEC);
    printf("[OK] Line\r\n");

    /* 엔코더 EXTI */
    Encoder_Init(&encoder,
                 GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_4, GPIO_PIN_15);
    printf("[OK] Encoder\r\n");

    HAL_Delay(500);
    printf("All systems GO.\r\n");
}


static void Gimbal_ControlLoop(void)
{
    if (MPU6050_ReadAll(&hi2c1, &mpu) != HAL_OK) return;
    Attitude_Update(&attitude, &mpu);

    if (system_mode == MODE_IDLE) {
        Servo_WriteAngle(&servo_pitch, 0.0f);
        Servo_WriteAngle(&servo_roll,  0.0f);
        return;
    }

    float u_pitch = PID_Compute(&pid_pitch, 0.0f, attitude.pitch);
    float u_roll  = PID_Compute(&pid_roll,  0.0f, attitude.roll);

    Servo_WriteAngle(&servo_pitch, u_pitch);
    Servo_WriteAngle(&servo_roll,  u_roll);
}


static void Vehicle_ControlLoop(void)
{
    switch (system_mode) {
        case MODE_IDLE:
        case MODE_GIMBAL_ONLY:
            Mecanum_Stop(&mecanum);
            break;

        case MODE_MANUAL_DRIVE:
            /* UART 명령에서 Mecanum_Drive 호출 — 여기서는 유지만 */
            break;

        case MODE_LINE_FOLLOW:
            LineFollow_Update(&line_follow);
            break;
    }

    /* 엔코더 방향 갱신 — 현재 PWM 부호로 추정 */
    int32_t r, l;

    r = __HAL_TIM_GET_COMPARE(wheel_fl.htim_rpwm, wheel_fl.ch_rpwm);
    l = __HAL_TIM_GET_COMPARE(wheel_fl.htim_lpwm, wheel_fl.ch_lpwm);
    Encoder_SetDirection(&encoder, 0, (float)(r - l));

    r = __HAL_TIM_GET_COMPARE(wheel_fr.htim_rpwm, wheel_fr.ch_rpwm);
    l = __HAL_TIM_GET_COMPARE(wheel_fr.htim_lpwm, wheel_fr.ch_lpwm);
    Encoder_SetDirection(&encoder, 1, (float)(r - l));

    r = __HAL_TIM_GET_COMPARE(wheel_rl.htim_rpwm, wheel_rl.ch_rpwm);
    l = __HAL_TIM_GET_COMPARE(wheel_rl.htim_lpwm, wheel_rl.ch_lpwm);
    Encoder_SetDirection(&encoder, 2, (float)(r - l));

    r = __HAL_TIM_GET_COMPARE(wheel_rr.htim_rpwm, wheel_rr.ch_rpwm);
    l = __HAL_TIM_GET_COMPARE(wheel_rr.htim_lpwm, wheel_rr.ch_lpwm);
    Encoder_SetDirection(&encoder, 3, (float)(r - l));
}


static void Process_UartCommand(void)
{
    uint8_t c = uart_rx_byte;

    switch (c) {
        case '0': system_mode = MODE_IDLE;          printf(">> IDLE\r\n");          return;
        case '1': system_mode = MODE_GIMBAL_ONLY;   printf(">> Gimbal only\r\n");   return;
        case '2': system_mode = MODE_MANUAL_DRIVE;  printf(">> Manual\r\n");        return;
        case '3': system_mode = MODE_LINE_FOLLOW;
                  LineFollow_Stop(&line_follow);
                  printf(">> Line follow\r\n"); return;
        case 'r': Encoder_Reset(&encoder); printf(">> Enc reset\r\n"); return;
    }

    if (system_mode != MODE_MANUAL_DRIVE) return;

    switch (c) {
        case 'w': Mecanum_MoveForward (&mecanum, 0.4f); printf("> FWD\r\n"); break;
        case 's': Mecanum_MoveBackward(&mecanum, 0.4f); printf("> BWD\r\n"); break;
        case 'a': Mecanum_StrafeLeft  (&mecanum, 0.4f); printf("> LEFT(strafe)\r\n"); break;
        case 'd': Mecanum_StrafeRight (&mecanum, 0.4f); printf("> RIGHT(strafe)\r\n"); break;
        case 'q': Mecanum_TurnLeft    (&mecanum, 0.4f); printf("> TURN L\r\n"); break;
        case 'e': Mecanum_TurnRight   (&mecanum, 0.4f); printf("> TURN R\r\n"); break;
        case ' ': Mecanum_Stop(&mecanum);               printf("> STOP\r\n"); break;
    }
}


static void Debug_Print(void)
{
    static const char *mode_name[] = { "IDLE", "GIMB", "MAN ", "LINE" };
    printf("[%s] P:%+6.2f R:%+6.2f | Line:%+5.2f%s | Enc[%5ld %5ld %5ld %5ld]\r\n",
           mode_name[system_mode],
           attitude.pitch, attitude.roll,
           line_sensor.position, line_sensor.line_lost ? "(LOST)" : "      ",
           (long)Encoder_GetCount(&encoder, 0),
           (long)Encoder_GetCount(&encoder, 1),
           (long)Encoder_GetCount(&encoder, 2),
           (long)Encoder_GetCount(&encoder, 3));
}


static void Error_Trap(const char *msg)
{
    printf("\r\nFATAL: %s\r\n", msg);
    Mecanum_Stop(&mecanum);
    Servo_WriteAngle(&servo_pitch, 0.0f);
    Servo_WriteAngle(&servo_roll,  0.0f);
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(100);
    }
}


/* ───── HAL 콜백 ───── */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if      (htim->Instance == TIM7) gimbal_flag  = 1;
    else if (htim->Instance == TIM6) vehicle_flag = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Encoder_OnInterrupt(&encoder, GPIO_Pin);
}

/* USER CODE END 4 */
