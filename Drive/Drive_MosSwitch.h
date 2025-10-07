#ifndef __DRIVE_MOSSWITCH_H__
#define __DRIVE_MOSSWITCH_H__

extern void Drive_MosSwitch_OFF(void);
extern void Drive_MosSwitch210_PWMOut(void);
extern void Drive_MosSwitch245_PWMOut(void);
extern void Drive_MosSwitch_SetDuty(uint16_t duty);
void Drive_MosPWMoutMode(void);

#define MAX_PWM_PRIOD 10000

#define T210_T115_GPIO_PIN GPIO_PIN_7
#define T210_T115_GPIO_PORT GPIOA

#define T245_GPIO_PIN GPIO_PIN_5
#define T245_GPIO_PORT GPIOA

#endif
