/**
 ******************************************************************************
 * @file    main.c
 * @author  MCU Application Team
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by Puya under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* define ------------------------------------------------------------*/
/* variables ---------------------------------------------------------*/
TYPEDEF_ALL_STATUS_S AllStatus_S = {
    .P_flashSave_s1 = (TYPEDEF_FLASHSAVE_S *)FLASH_USER_START_ADDR1,
    .P_flashSave_s2 = (TYPEDEF_FLASHSAVE_S *)FLASH_USER_START_ADDR2,
    .P_flashSave_s3 = (TYPEDEF_FLASHSAVE_S *)FLASH_USER_START_ADDR3};

/* user code ---------------------------------------------------------*/
/* macro -------------------------------------------------------------*/
/* function prototypes -----------------------------------------------*/

/**
 * @brief  Main program.
 * @retval int
 */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();
  HAL_Delay(400);                                               // 等待电源稳定
  Drive_DisplayLcd_Gpio_Init();                                 // 初始化LCD引脚
  Drive_DisplayLcd_Init();                                      // 初始化LCD
  Drive_BackLed_Init();                                         // 初始化背光
  Drive_Buz_Init();                                             // 初始化蜂鸣器
  Drive_SystemClockConfig(RCC_HSICALIBRATION_24MHz);            // 初始化系统时钟
  HAL_Delay(25);                                                // 等待电源稳定
  Drive_FlashSaveInit();                                        // 初始化历史温度
  Drive_BUTTON_Init();                                          // 初始化按键
  Drive_MosSwitch_OFF();                                        // 初始化mos管引脚
  Drive_AdcGpio_init();                                         // 初始化ADC引脚
  Drive_AdcConfig();                                            // 初始化ADC
  Drive_Lcd_AllIcon_init();                                     // 初始化图标
  Drive_Encoder_Init();                                         // 初始化旋转编码器
  APP_shortCircuitProtection();                                 // 短路&开路判断
  Drive_Iwdg_init();                                            // 初始化看门狗
  Drive_Buz_OnOff(BUZ_20MS, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE); // 启动音
  while (1)
  {
    app_timeSlice_Task(); // 时间片任务运行

    /* AllStatus_S.adc_value[SOLDERING_TEMP210_NUM] = Drive_ADCConvert(SOLDERING_TEMP210_NUM);
    AllStatus_S.adc_filter_value = (uint16_t)APP_FirFilter_ADC((float32_t)AllStatus_S.adc_value[SOLDERING_TEMP210_NUM]);
    Lcd_smgDowm3_SetHex */

    /* AllStatus_S.adc_value[SOLDERING_TID_NUM] = Drive_ADCConvert(SOLDERING_TID_NUM);
    Lcd_smgDowm3_SetHex(AllStatus_S.adc_value[SOLDERING_TID_NUM]); */

    /* HAL_GPIO_WritePin(T245_GPIO_PORT, T245_GPIO_PIN, GPIO_PIN_SET);
    AllStatus_S.adc_value[SOLDERING_ELECTRICITY_NUM] = Drive_ADCConvert(SOLDERING_ELECTRICITY_NUM);
    HAL_GPIO_WritePin(T245_GPIO_PORT, T245_GPIO_PIN, GPIO_PIN_RESET);
    Lcd_smgDowm3_SetHex(AllStatus_S.adc_value[SOLDERING_ELECTRICITY_NUM]); */
  

    /* HAL_Delay(50);
    Drive_DisplayLcd_sendData_Task(); // LCD数据发送任务 */
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void APP_ErrorHandler(void)
{
  static uint8_t oneSttate = 0;
  Drive_MosSwitch_OFF();
  Lcd_icon_onOff(icon_400Temp, 1);
  Lcd_icon_onOff(icon_350Temp, 1);
  Lcd_icon_onOff(icon_300Temp, 1);
  Lcd_icon_onOff(icon_SET, 1);
  Lcd_icon_onOff(icon_temp, 0);
  Lcd_icon_onOff(icon_cup, 0); // 熄灭
  Lcd_smgUp3_SetNum(0);
  while (1)
  {
    HAL_IWDG_Refresh(&IwdgHandle);

    switch (AllStatus_S.SolderingState)
    {
    case SOLDERING_STATE_INIT_ERROR:
      Lcd_smgDowm3_SetErrorNum(ERROR_SYSTEM_INIT, 1);
      break;
    case SOLDERING_STATE_SHORTCIR_ERROR:
      Lcd_smgDowm3_SetErrorNum(ERROR_E2, 1);
      break;
    case SOLDERING_STATE_OPEN_ERROR:
      Lcd_smgDowm3_SetErrorNum(ERROR_E0, 1);
      break;
    case SOLDERING_STATE_PULL_OUT_ERROR:
      Lcd_smgDowm3_SetErrorNum(ERROR_E3, 1);
      APP_shortCircuitProtection();
      if (AllStatus_S.SolderingState == SOLDERING_STATE_OK)
        return;
      break;
    case SOLDERING_STATE_NTC_ERROR:
      Lcd_smgDowm3_SetErrorNum(ERROR_E1, 1);
      break;
    }
    Drive_DisplayLcd_sendData_Task();
    if (!oneSttate)
    {
      Drive_Buz_OnOff(BUZ_20MS, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
      oneSttate = 1;
    }
    HAL_Delay(500);
    Lcd_smgDowm3_SetErrorNum(0, 0);
    Drive_DisplayLcd_sendData_Task();
    HAL_Delay(500);
    Drive_Buz_OnOff(BUZ_20MS, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line)  */
  /* infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
