/**
 * @file    linesensor.c
 */
#include "linesensor.h"

void LineSensor_Init(LineSensor_t *ls,
                     GPIO_TypeDef *p1, uint16_t pin1,
                     GPIO_TypeDef *p2, uint16_t pin2,
                     GPIO_TypeDef *p3, uint16_t pin3,
                     GPIO_TypeDef *p4, uint16_t pin4,
                     GPIO_TypeDef *p5, uint16_t pin5)
{
    ls->port[0] = p1; ls->pin[0] = pin1;
    ls->port[1] = p2; ls->pin[1] = pin2;
    ls->port[2] = p3; ls->pin[2] = pin3;
    ls->port[3] = p4; ls->pin[3] = pin4;
    ls->port[4] = p5; ls->pin[4] = pin5;

    for (int i = 0; i < LINE_SENSOR_COUNT; i++) ls->raw[i] = 0;
    ls->position = 0.0f;
    ls->line_lost = false;
    ls->last_known_position = 0.0f;
}

void LineSensor_Read(LineSensor_t *ls)
{
    /* 1) 5개 핀 읽기 + 극성 처리
     *    on_line = 1 이면 검은선 위 */
    int sum_b = 0;     // 검은선 검출 센서 개수
    float weighted = 0.0f;

    /* 가중치: -2, -1, 0, +1, +2 (왼→오른) */
    static const float weight[LINE_SENSOR_COUNT] = {-2.0f, -1.0f, 0.0f, +1.0f, +2.0f};

    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        GPIO_PinState s = HAL_GPIO_ReadPin(ls->port[i], ls->pin[i]);
#if LINE_ACTIVE_LOW
        ls->raw[i] = (s == GPIO_PIN_RESET) ? 1 : 0;  // LOW면 검은선
#else
        ls->raw[i] = (s == GPIO_PIN_SET)   ? 1 : 0;
#endif
        if (ls->raw[i]) {
            weighted += weight[i];
            sum_b++;
        }
    }

    /* 2) 라인 위치 계산 */
    if (sum_b == 0) {
        /* 라인 잃음 → 마지막 위치 유지하고 플래그 set */
        ls->line_lost = true;
        ls->position = ls->last_known_position;
    } else {
        ls->line_lost = false;
        ls->position = weighted / (float)sum_b;
        ls->last_known_position = ls->position;
    }
}
