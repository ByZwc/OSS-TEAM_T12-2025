#include "main.h"

void Drive_Buz_Init(void)
{
#if (USE_BUZ_TYPE == 0)

  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the GPIO_BUZ Clock */
  BUZ_GPIO_CLK_ENABLE;

  /* Configure the GPIO_BUZ pin */
  GPIO_InitStruct.Pin = BUZ_GPIO_PIN;
  GPIO_InitStruct.Mode = BUZ_GPIO_MODE;
  GPIO_InitStruct.Pull = BUZ_GPIO_PULL;
  GPIO_InitStruct.Speed = BUZ_GPIO_SPEED;

  HAL_GPIO_Init(BUZ_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BUZ_GPIO_PORT, BUZ_GPIO_PIN, BUZ_GPIO_RESET);

#else

  Drive_BuzMusic_Init();
#endif
}

TIM_HandleTypeDef TimHandle_16;
TIM_OC_InitTypeDef sConfig_16;

/**
 * @brief  Initialize TIM16
 * @param  None
 * @retval None
 */
static void Drive_TIM16_INIT(void)
{
  __HAL_RCC_TIM16_CLK_ENABLE();

  /* Select TIM1 */
  TimHandle_16.Instance = TIM16;

  /* Auto-reload value */
  TimHandle_16.Init.Period = 100;

  /* Prescaler value */
  TimHandle_16.Init.Prescaler = 6 - 1;

  /* Clock not divided */
  TimHandle_16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  /* Up-counting mode*/
  TimHandle_16.Init.CounterMode = TIM_COUNTERMODE_UP;

  /* No repetition */
  TimHandle_16.Init.RepetitionCounter = 1 - 1;

  /* Auto-reload register not buffered */
  TimHandle_16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  /* Initialize clock settings */
  if (HAL_TIM_Base_Init(&TimHandle_16) != HAL_OK)
  {
    AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
    APP_ErrorHandler();
  }
}

/**
 * @brief  TIM16 PWM Configuration
 * @param  None
 * @retval None
 */
static void Drive_TIM16_PWM(void)
{
  /*Output configured for PWM1 mode */
  sConfig_16.OCMode = TIM_OCMODE_PWM1;

  /*OC channel output active high */
  sConfig_16.OCPolarity = TIM_OCPOLARITY_HIGH;

  /*Disable the output compare fast mode */
  sConfig_16.OCFastMode = TIM_OCFAST_DISABLE;

  /*OCN channel output active high */
  sConfig_16.OCNPolarity = TIM_OCNPOLARITY_HIGH;

  /*Idle state OC1N output low level */
  sConfig_16.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  /*Idle state OC1 output low level*/
  sConfig_16.OCIdleState = TIM_OCIDLESTATE_RESET;

  /*Set CC2 pulse value to 20, resulting in a duty cycle of 20/50 = 40% */
  sConfig_16.Pulse = 0;

  /* Configure Channel 2 for PWM */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle_16, &sConfig_16, TIM_CHANNEL_1) != HAL_OK)
  {
    AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
    APP_ErrorHandler();
  }
}

void Drive_BuzMusic_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the GPIO_LED Clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_TIM16;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  Drive_TIM16_INIT();

  Drive_TIM16_PWM();

  /* Start PWM output on Channel 2 */
  if (HAL_TIM_PWM_Start(&TimHandle_16, TIM_CHANNEL_1) != HAL_OK)
  {
    AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
    APP_ErrorHandler();
  }
}
