#ifndef __DRIVE_BACKLED_H__
#define __DRIVE_BACKLED_H__

#define BACKLED_GPIO_PORT      GPIOB
#define BACKLED_GPIO_PIN       GPIO_PIN_5
#define BACKLED_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define BACKLED_GPIO_MODE      GPIO_MODE_OUTPUT_PP
#define BACKLED_GPIO_PULL      GPIO_NOPULL
#define BACKLED_GPIO_SPEED     GPIO_SPEED_FREQ_HIGH

extern void Drive_BackLed_Init(void);

void Drive_BackLed_OnOff(uint8_t OnOff);

#endif

