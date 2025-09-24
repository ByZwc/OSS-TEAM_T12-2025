#include "main.h"

void Drive_BUTTON_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  DRIVE_BUTTON_RCC_ENABLE();
  // SLEEP_BUTTON_RCC_ENABLE();

  GPIO_InitStruct.Pin = DRIVE_BUTTON_GPIO_PIN;
  GPIO_InitStruct.Pull = DRIVE_BUTTON_GPIO_PULL;
  GPIO_InitStruct.Speed = DRIVE_BUTTON_GPIO_SPEED;
  GPIO_InitStruct.Mode = DRIVE_BUTTON_GPIO_MODE;

  HAL_GPIO_Init(DRIVE_BUTTON_GPIO_PORT, &GPIO_InitStruct);

  /* GPIO_InitStruct.Pin = SLEEP_BUTTON_GPIO_PIN;
  GPIO_InitStruct.Pull = SLEEP_BUTTON_GPIO_PULL;
  HAL_GPIO_Init(SLEEP_BUTTON_GPIO_PORT, &GPIO_InitStruct); */
}

uint32_t Drive_BUTTON_GetState(void)
{
  return !(HAL_GPIO_ReadPin(DRIVE_BUTTON_GPIO_PORT, DRIVE_BUTTON_GPIO_PIN));
}

/* uint32_t Drive_Sleep_GetState(void)
{
  return !(HAL_GPIO_ReadPin(SLEEP_BUTTON_GPIO_PORT, SLEEP_BUTTON_GPIO_PIN));
} */
