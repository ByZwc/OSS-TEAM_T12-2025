#ifndef __DRIVE_BUZ_H__
#define __DRIVE_BUZ_H__

// Macro definitions for easy modification
#define BUZ_GPIO_PORT        GPIOA
#define BUZ_GPIO_PIN         GPIO_PIN_6
#define BUZ_GPIO_CLK_ENABLE  __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUZ_GPIO_MODE        GPIO_MODE_OUTPUT_PP
#define BUZ_GPIO_PULL        GPIO_NOPULL
#define BUZ_GPIO_SPEED       GPIO_SPEED_FREQ_HIGH
#define BUZ_GPIO_RESET       GPIO_PIN_RESET

extern void Drive_Buz_Init(void);
void Drive_BuzMusic_Init(void);
#endif


