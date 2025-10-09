#include "main.h"

void Drive_BackLed_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the GPIO_LED Clock */
  BACKLED_GPIO_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = BACKLED_GPIO_PIN;
  GPIO_InitStruct.Mode = BACKLED_GPIO_MODE;
  GPIO_InitStruct.Pull = BACKLED_GPIO_PULL;
  GPIO_InitStruct.Speed = BACKLED_GPIO_SPEED;

  HAL_GPIO_Init(BACKLED_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BACKLED_GPIO_PORT, BACKLED_GPIO_PIN, GPIO_PIN_SET);
}

void Drive_BackLed_PWMOut(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  BACKLED_GPIO_CLK_ENABLE();

  GPIO_InitStruct.Pin = BACKLED_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;

  HAL_GPIO_Init(BACKLED_GPIO_PORT, &GPIO_InitStruct);
  Drive_MosPWMoutMode(6 - 1);

  TIM3->CCR2 = 2000;
}

void Drive_BackLed_OnOff(uint8_t OnOff)
{
  if (OnOff)
    HAL_GPIO_WritePin(BACKLED_GPIO_PORT, BACKLED_GPIO_PIN, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(BACKLED_GPIO_PORT, BACKLED_GPIO_PIN, GPIO_PIN_RESET);
}
