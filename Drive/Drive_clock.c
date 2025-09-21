#include "main.h"

void Drive_SystemClockConfig(uint32_t HSICalibrationValue)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure HSI, HSE, LSI clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                     /* Enable HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                     /* HSI prescaler */
  RCC_OscInitStruct.HSICalibrationValue = HSICalibrationValue; /* Set HSI output clock as 24MHz, the library will set the calibration value */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                    /* Disable HSE */
  RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;                /* HSE frequency range */
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;                     /* Enable LSI */

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
    APP_ErrorHandler();
  }

  /* Initialize AHB, and APB bus clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; /* SYSCLK source is HSI */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     /* Setting the AHB prescaler */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      /* Setting the APB1 prescaler */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
    APP_ErrorHandler();
  }
}
