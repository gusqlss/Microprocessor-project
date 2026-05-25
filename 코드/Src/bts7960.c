#include "bts7960.h"

void BTS7960_Init(BTS7960_t *m,
                  TIM_HandleTypeDef *htim_rpwm, uint32_t ch_rpwm,
                  GPIO_TypeDef *gpio_lpwm, uint16_t pin_lpwm,
                  uint32_t arr, int reverse)
{
    m->htim_rpwm = htim_rpwm;
    m->ch_rpwm   = ch_rpwm;
    m->gpio_lpwm = gpio_lpwm;
    m->pin_lpwm  = pin_lpwm;
    m->arr       = arr;
    m->reverse   = reverse;

    /* 초기 상태: 정지 */
    if(m->htim_rpwm != NULL) {
        __HAL_TIM_SET_COMPARE(m->htim_rpwm, m->ch_rpwm, 0);
        HAL_TIM_PWM_Start(m->htim_rpwm, m->ch_rpwm);
    }
    HAL_GPIO_WritePin(m->gpio_lpwm, m->pin_lpwm, GPIO_PIN_RESET);
}

void BTS7960_SetSpeed(BTS7960_t *m, float speed)
{
    if (speed >  1.0f) speed =  1.0f;
    if (speed < -1.0f) speed = -1.0f;

    if (m->reverse) speed = -speed;

    uint32_t duty = (uint32_t)(m->arr * (speed >= 0 ? speed : -speed));

    if (speed > 0.0f) {
        // 정방향: PWM 출력 공급, GPIO 로우
        if(m->htim_rpwm != NULL) __HAL_TIM_SET_COMPARE(m->htim_rpwm, m->ch_rpwm, duty);
        HAL_GPIO_WritePin(m->gpio_lpwm, m->pin_lpwm, GPIO_PIN_RESET);
    } else if (speed < 0.0f) {
        // 역방향: PWM은 최대치 브레이크 상태 유도 혹은 역방향 제어 로직에 맞춰 핀 반전
        if(m->htim_rpwm != NULL) __HAL_TIM_SET_COMPARE(m->htim_rpwm, m->ch_rpwm, m->arr - duty);
        HAL_GPIO_WritePin(m->gpio_lpwm, m->pin_lpwm, GPIO_PIN_SET);
    } else {
        // 정지
        if(m->htim_rpwm != NULL) __HAL_TIM_SET_COMPARE(m->htim_rpwm, m->ch_rpwm, 0);
        HAL_GPIO_WritePin(m->gpio_lpwm, m->pin_lpwm, GPIO_PIN_RESET);
    }
}

void BTS7960_Stop(BTS7960_t *m)
{
    BTS7960_SetSpeed(m, 0.0f);
}
