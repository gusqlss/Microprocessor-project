/**
 * @file    linefollow.c
 */
#include "linefollow.h"

void LineFollow_Init(LineFollow_t *lf,
                     LineSensor_t *sensor, Mecanum_t *mecanum,
                     float base_speed,
                     float Kp, float Ki, float Kd,
                     float dt_sec)
{
    lf->sensor = sensor;
    lf->mecanum = mecanum;
    lf->base_speed = base_speed;
    lf->turn_gain = 1.0f;
    lf->stop_on_lost = true;

    /* PID: 출력 ±1.0 (회전 명령은 -1~+1), 적분 한계 ±0.5 */
    PID_Init(&lf->pid, Kp, Ki, Kd, -1.0f, 1.0f, 0.5f, dt_sec);
}

void LineFollow_Update(LineFollow_t *lf)
{
    /* 1) 센서 읽기 */
    LineSensor_Read(lf->sensor);

    /* 2) 라인 잃음 처리 */
    if (lf->sensor->line_lost) {
        if (lf->stop_on_lost) {
            Mecanum_Stop(lf->mecanum);
            return;
        }
        /* 또는: 직전 위치 부호로 천천히 그쪽으로 회전 탐색하는 로직 가능
         *      여기선 단순화하여 정지 */
    }

    /* 3) PID 계산
     *    setpoint = 0 (라인이 중앙에 와야 함)
     *    measurement = 라인 위치 (-2.0 ~ +2.0)
     *    출력 = 회전 명령 ω (양수 = 좌회전)
     *
     *    라인이 오른쪽(position > 0)에 있으면 우회전(ω < 0)으로 보정해야 함
     *    error = 0 - position = -position 이므로
     *    PID 출력이 그대로 ω로 들어가면 부호가 맞음 (확인 필수)
     */
    float wz = PID_Compute(&lf->pid, 0.0f, lf->sensor->position);
    wz *= lf->turn_gain;

    /* 4) 메카넘에 명령 — 전진 + 회전 보정 */
    Mecanum_Drive(lf->mecanum, lf->base_speed, 0.0f, wz);
}

void LineFollow_Stop(LineFollow_t *lf)
{
    Mecanum_Stop(lf->mecanum);
    PID_Reset(&lf->pid);
}
