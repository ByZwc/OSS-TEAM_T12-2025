#include "main.h"

ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef sConfig_ADC = {0};
/**
 * @brief  ADC configuration function
 * @param  None
 * @retval None
 */
void Drive_AdcConfig(void)
{
    __HAL_RCC_ADC_FORCE_RESET();
    __HAL_RCC_ADC_RELEASE_RESET();
    __HAL_RCC_ADC_CLK_ENABLE();

    AdcHandle.Instance = ADC1;
    /* ADC calibration */
    if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
    {
        AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
        APP_ErrorHandler();
    }

    AdcHandle.Instance = ADC1;
    AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;            /* Set ADC clock */
    AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;                      /* 12-bit resolution for converted data */
    AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                      /* Right-alignment for converted data */
    AdcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;            /* Scan sequence direction */
    AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;                   /* Single sampling  */
    AdcHandle.Init.LowPowerAutoWait = ENABLE;                            /* Enable wait for conversion mode */
    AdcHandle.Init.ContinuousConvMode = DISABLE;                         /* Single conversion mode */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;                      /* Disable discontinuous mode */
    AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;                /* Software triggering */
    AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* No external trigger edge */
    AdcHandle.Init.DMAContinuousRequests = DISABLE;                      /* isable DMA */
    AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                   /* Overrun handling: overwrite previous value  */
    AdcHandle.Init.SamplingTimeCommon = ADC_SAMPLETIME_71CYCLES_5;       /* Set sampling time */
    /* Initialize ADC */
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
    {
        AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
        APP_ErrorHandler();
    }
}

void Drive_AdcGpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = DRIVE_ADC_GPIO_PINS;
    GPIO_InitStruct.Pull = DRIVE_ADC_GPIO_PULL;
    GPIO_InitStruct.Mode = DRIVE_ADC_GPIO_MODE;

    HAL_GPIO_Init(DRIVE_ADC_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief  ADC convert function
 * @param  Channel: The channel to be converted
 * @retval ADC converted value
 */
uint32_t Drive_ADCConvert(uint32_t Channel)
{
    static uint32_t last_Channel = 0;
    uint16_t adcvalue;

    if (last_Channel != Channel)
    {
        last_Channel = Channel;
        /* Clear all channels */
        WRITE_REG(ADC1->CHSELR, 0);

        /* Config selected channels */
        sConfig_ADC.Rank = ADC_RANK_CHANNEL_NUMBER;
        sConfig_ADC.Channel = Channel;
        if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig_ADC) != HAL_OK)
        {
            AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
            APP_ErrorHandler();
        }
    }

    /* ADC Start */
    HAL_ADC_Start(&AdcHandle);

    /* Polling for ADC Conversion */
    HAL_ADC_PollForConversion(&AdcHandle, 10);

    /* Get ADC Value */
    adcvalue = HAL_ADC_GetValue(&AdcHandle);

    return (uint32_t)adcvalue;
}
