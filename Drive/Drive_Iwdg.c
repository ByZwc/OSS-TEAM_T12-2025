#include "main.h"

IWDG_HandleTypeDef IwdgHandle;

void Drive_Iwdg_init(void)
{
    __HAL_RCC_LSI_ENABLE();
    IwdgHandle.Instance = IWDG;                    /* Select IWDG */
    IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32; /* Configure prescaler to 32 */
    IwdgHandle.Init.Reload = (0xFF0);              /* Set IWDG counter reload value to 1024, 1s */
    /* Initialize IWDG */
    if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
    {
        AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
        APP_ErrorHandler();
    }
}
