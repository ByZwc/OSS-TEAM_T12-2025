#include "main.h"



void Drive_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE(); /* Enable GPIOA clock */

    GPIO_InitStruct.Pin = ENCODER_PIN_A;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(ENCODER_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pin = ENCODER_PIN_B;
    HAL_GPIO_Init(ENCODER_GPIO_PORT, &GPIO_InitStruct);

    /* Enable EXTI interrupt */
    HAL_NVIC_EnableIRQ(ENCODER_IRQ);

    /* Configure interrupt priority */
    HAL_NVIC_SetPriority(ENCODER_IRQ, ENCODER_IRQ_PRIORITY, ENCODER_IRQ_SUBPRIORITY);
}


#define ENCODER_DEBOUNCE_MS 100

void EXTI4_15_IRQHandler(void)
{
    static uint8_t last_direction = 0;
    static uint32_t last_tick = 0;
    uint8_t curr_direction;

    // 100ms内触发两次则清空中断并立即返回，每100ms只允许触发一次
    if ((uwTick - last_tick) < ENCODER_DEBOUNCE_MS)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(ENCODER_PIN_B);
        return;
    }

    AllStatus_S.encoder_s.EncoderIsRun = 1;

    // 判断当前方向
    if (HAL_GPIO_ReadPin(ENCODER_GPIO_PORT, ENCODER_PIN_A) == HAL_GPIO_ReadPin(ENCODER_GPIO_PORT, ENCODER_PIN_B))
        curr_direction = 1; // 顺时针
    else
        curr_direction = 0; // 逆时针

    // 如果距离上次旋转超过ENCODER_DEBOUNCE_MS，重置方向
    if ((uwTick - last_tick) > ENCODER_DEBOUNCE_MS)
    {
        last_direction = curr_direction;
    }

    // 如果方向发生变化但在ENCODER_DEBOUNCE_MS内，保持原方向
    if (curr_direction != last_direction && (uwTick - last_tick) <= ENCODER_DEBOUNCE_MS)
    {
        AllStatus_S.encoder_s.TurnleftOrRight = last_direction;
    }
    else
    {
        AllStatus_S.encoder_s.TurnleftOrRight = curr_direction;
        last_direction = curr_direction;
    }

    last_tick = uwTick;

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
}
