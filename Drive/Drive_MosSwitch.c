#include "main.h"

TIM_HandleTypeDef TimHandle;
TIM_OC_InitTypeDef sConfig;

void Drive_MosSwitch_OFF(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable the GPIO_LED Clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure the GPIO_LED pin */
    GPIO_InitStruct.Pin = T245_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(T245_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(T245_GPIO_PORT, T245_GPIO_PIN, GPIO_PIN_RESET);

}

/**
 * @brief  Initialize TIM1
 * @param  None
 * @retval None
 */
static void Drive_TIM3_INIT(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Select TIM1 */
    TimHandle.Instance = TIM3;

    /* Auto-reload value */
    TimHandle.Init.Period = MAX_PWM_PRIOD;

    /* Prescaler value */
    TimHandle.Init.Prescaler = 6 - 1;

    /* Clock not divided */
    TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    /* Up-counting mode*/
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

    /* No repetition */
    TimHandle.Init.RepetitionCounter = 1 - 1;

    /* Auto-reload register not buffered */
    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* Initialize clock settings */
    if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
    {
        AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
        APP_ErrorHandler();
    }
}
/**
 * @brief  TIM1 PWM Configuration
 * @param  None
 * @retval None
 */
static void Drive_TIM3_PWM(void)
{
    /*Output configured for PWM1 mode */
    sConfig.OCMode = TIM_OCMODE_PWM1;

    /*OC channel output active high */
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;

    /*Disable the output compare fast mode */
    sConfig.OCFastMode = TIM_OCFAST_DISABLE;

    /*OCN channel output active high */
    sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;

    /*Idle state OC1N output low level */
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    /*Idle state OC1 output low level*/
    sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;

    /*Set CC2 pulse value to 20, resulting in a duty cycle of 20/50 = 40% */
    sConfig.Pulse = 0;

    /* Configure Channel 2 for PWM */
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
    {
        AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
        APP_ErrorHandler();
    }
}

static void Drive_MosPWMoutMode(void)
{
    Drive_TIM3_INIT();

    Drive_TIM3_PWM();

    /* Start PWM output on Channel 2 */
    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
    {
        AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
        APP_ErrorHandler();
    }
}

void Drive_MosSwitch210_PWMOut(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Drive_MosSwitch_OFF();

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
    if (!AllStatus_S.PwmIsInitComplete)
    {
        Drive_MosPWMoutMode();
        AllStatus_S.PwmIsInitComplete = 1;
    }
}

void Drive_MosSwitch245_PWMOut(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Drive_MosSwitch_OFF();

    GPIO_InitStruct.Pin = T245_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_TIM3;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
    if (!AllStatus_S.PwmIsInitComplete)
    {
        Drive_MosPWMoutMode();
        AllStatus_S.PwmIsInitComplete = 1;
    }
}

void Drive_MosSwitch_SetDuty(uint16_t duty)
{
    TIM3->CCR2 = duty;
}
