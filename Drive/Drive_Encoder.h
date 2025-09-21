#ifndef __DRIVE_ENCODER_H__
#define __DRIVE_ENCODER_H__

#define ENCODER_GPIO_PORT GPIOB
#define ENCODER_PIN_A GPIO_PIN_7
#define ENCODER_PIN_B GPIO_PIN_6
#define ENCODER_IRQ EXTI4_15_IRQn
#define ENCODER_IRQ_PRIORITY 0
#define ENCODER_IRQ_SUBPRIORITY 0

extern void Drive_Encoder_Init(void);

#endif



