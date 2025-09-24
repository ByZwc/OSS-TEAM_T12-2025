#ifndef __DRIVE_ADC_H__
#define __DRIVE_ADC_H__


#define NTC_ADC_PIN GPIO_PIN_0
#define ELECTRICITY_ADC_PIN GPIO_PIN_1
#define SOLDERING_TEMP_ADC_PIN GPIO_PIN_2
#define TID_ADC_PIN GPIO_PIN_4
#define SLEEP_ADC_PIN GPIO_PIN_7

#define DRIVE_ADC_GPIO_PORT GPIOA
#define DRIVE_ADC_GPIO_PINS (NTC_ADC_PIN | ELECTRICITY_ADC_PIN | SOLDERING_TEMP_ADC_PIN | TID_ADC_PIN | SLEEP_ADC_PIN)
#define DRIVE_ADC_GPIO_PULL GPIO_NOPULL
#define DRIVE_ADC_GPIO_MODE GPIO_MODE_ANALOG

extern void Drive_AdcConfig(void);
extern uint32_t Drive_ADCConvert(uint32_t Channel);
extern void Drive_AdcGpio_init(void);

#endif


