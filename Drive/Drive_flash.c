#include "main.h"

/**
 * @brief  Erase Flash
 * @param  None
 * @retval None
 */
void Drive_FlashErase(uint32_t addr)
{
    uint32_t PAGEError = 0;
    FLASH_EraseInitTypeDef EraseInitStruct = {0};

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGEERASE; /* Erase type: FLASH_TYPEERASE_PAGEERASE = Page erase, FLASH_TYPEERASE_SECTORERASE = Sector erase */
    EraseInitStruct.PageAddress = addr;                    /* Starting address for erase */
    // EraseInitStruct.NbPages = sizeof(AllStatus_S.flashSave_s) / FLASH_PAGE_SIZE; /* Number of pages to be erased */
    EraseInitStruct.NbPages = 1;                                   /* Number of pages to be erased */
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) /* Perform page erase, PAGEError returns the page with erase error, returns 0xFFFFFFFF for successful erase */
    {
        AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
        APP_ErrorHandler();
    }
}

void Drive_FlashProgram(uint32_t addr)
{
    uint32_t flash_program_start = addr;
    uint32_t *src = (uint32_t *)&AllStatus_S.flashSave_s;
    uint32_t total_size = sizeof(AllStatus_S.flashSave_s);
    uint32_t page_size = FLASH_PAGE_SIZE;
    uint32_t pad_size = (total_size % page_size) ? (page_size - (total_size % page_size)) : 0;
    uint32_t write_size = total_size + pad_size;
    uint32_t temp_buf[FLASH_PAGE_SIZE / 4];
    uint32_t offset = 0;

    while (offset < write_size)
    {
        // 拷贝一页数据，剩余部分补0xFFFFFFFF
        for (uint32_t i = 0; i < FLASH_PAGE_SIZE / 4; i++)
        {
            if ((offset + i * 4) < total_size)
                temp_buf[i] = *((uint32_t *)((uint8_t *)src + offset + i * 4));
            else
                temp_buf[i] = 0xFFFFFFFF;
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, flash_program_start + offset, temp_buf) != HAL_OK)
        {
            // 写入失败，处理错误
            AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
            APP_ErrorHandler();
            break;
        }
        offset += FLASH_PAGE_SIZE;
    }
}

/**
 * @brief  Check if Flash is blank
 * @param  None
 * @retval None
 */
void Drive_FlashBlank(uint32_t Addr)
{
    uint32_t addr = 0;

    while (addr < sizeof(AllStatus_S.flashSave_s))
    {
        if (0xFFFFFFFF != HW32_REG(Addr + addr))
        {
            AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
            APP_ErrorHandler();
        }
        addr += 4;
    }
}

static void Drive_FlashProgramWithCheck(uint32_t addr)
{
    uint8_t try_count = 0;
    do
    {
        Drive_FlashProgram(addr);
        // 写入后校验
        if (memcmp((const void *)addr, (const void *)&AllStatus_S.flashSave_s, sizeof(AllStatus_S.flashSave_s)) == 0)
        {
            return; // 校验成功
        }
        try_count++;
    } while (try_count < 2);

    // 两次都失败则报错
    /* AllStatus_S.SolderingState = SOLDERING_STATE_INIT_ERROR;
    APP_ErrorHandler(); */
}

void Drive_FlashSaveData_addr(uint32_t addr)
{
    uint32_t primask_bit;
    /* Enter critical section */
    primask_bit = __get_PRIMASK();
    __disable_irq();

    HAL_IWDG_Refresh(&IwdgHandle);

    /* Unlock FLASH */
    HAL_FLASH_Unlock();

    /* Erase FLASH */
    Drive_FlashErase(addr);

    /* Check if FLASH is blank */
    Drive_FlashBlank(addr);

    /* program FLASH */
    Drive_FlashProgramWithCheck(addr);

    /* 锁定FLASH */
    HAL_FLASH_Lock();

    /* Exit critical section: restore previous priority mask */
    __set_PRIMASK(primask_bit);
}

void Drive_FlashSaveData(void)
{
    AllStatus_S.flashSave_s.SaveNum++;
    Drive_FlashSaveData_addr(FLASH_USER_START_ADDR1);
    Drive_FlashSaveData_addr(FLASH_USER_START_ADDR2);
    Drive_FlashSaveData_addr(FLASH_USER_START_ADDR3);
}

/**
 * @brief  Get the index of the maximum value among three numbers
 * @param  a First number
 * @param  b Second number
 * @param  c Third number
 * @retval Index of the maximum value (0 for a, 1 for b, 2 for c)
 */
static uint8_t app_getMaxOfnum(uint32_t a, uint32_t b, uint32_t c)
{
    if (a >= b && a >= c)
        return 0;
    else if (b >= a && b >= c)
        return 1;
    else
        return 2;
}

static void Drive_FlashRed(TYPEDEF_FLASHSAVE_S *P_flashSave_s)
{
    AllStatus_S.flashSave_s.checkVlue = FLASH_CHECK_VLUEb;
    AllStatus_S.flashSave_s.TarTemp = P_flashSave_s->TarTemp;
    AllStatus_S.Old_TarTemp = AllStatus_S.flashSave_s.TarTemp;
    AllStatus_S.flashSave_s.BuzOnOff = P_flashSave_s->BuzOnOff;
    AllStatus_S.flashSave_s.calibration_temp = P_flashSave_s->calibration_temp;
    AllStatus_S.flashSave_s.PreinstallTempOnOff = P_flashSave_s->PreinstallTempOnOff;
    AllStatus_S.flashSave_s.PreinstallTempNum = P_flashSave_s->PreinstallTempNum;
    AllStatus_S.flashSave_s.BackgroundLightOnoff = P_flashSave_s->BackgroundLightOnoff;
    AllStatus_S.flashSave_s.T245PowerCompensation = P_flashSave_s->T245PowerCompensation;
    AllStatus_S.flashSave_s.SaveNum = P_flashSave_s->SaveNum;
    AllStatus_S.flashSave_s.DisplayPowerOnOff = P_flashSave_s->DisplayPowerOnOff;
    if (AllStatus_S.flashSave_s.TarTemp < (MIN_TAR_TEMP - 5) || AllStatus_S.flashSave_s.TarTemp > MAX_TAR_TEMP)
        AllStatus_S.flashSave_s.TarTemp = FIRST_SOLDERING_TEMP;
    if (AllStatus_S.flashSave_s.calibration_temp < (CALIBRATION_TEMP_MIN) || AllStatus_S.flashSave_s.calibration_temp > (CALIBRATION_TEMP_MAX))
        AllStatus_S.flashSave_s.calibration_temp = 0;
    if (AllStatus_S.flashSave_s.T245PowerCompensation > T245_POWER_COMPENSATION_MAX)
        AllStatus_S.flashSave_s.T245PowerCompensation = 0;
    if (AllStatus_S.flashSave_s.PreinstallTempNum > 4)
        AllStatus_S.flashSave_s.PreinstallTempNum = 1;
    if (AllStatus_S.flashSave_s.PreinstallTempOnOff)
    {
        switch (AllStatus_S.flashSave_s.PreinstallTempNum)
        {
        case PREINSTALL_TMEP300:
            AllStatus_S.flashSave_s.TarTemp = 300;
            break;
        case PREINSTALL_TMEP350:
            AllStatus_S.flashSave_s.TarTemp = 350;
            break;
        case PREINSTALL_TMEP400:
            AllStatus_S.flashSave_s.TarTemp = 400;
            break;
        }
    }
}

static void Drive_FlashVerify(void)
{
    uint8_t max_index;
    if ((FLASH_CHECK_VLUEb != AllStatus_S.P_flashSave_s1->checkVlue) ||
        (FLASH_CHECK_VLUEb != AllStatus_S.P_flashSave_s2->checkVlue) ||
        (FLASH_CHECK_VLUEb != AllStatus_S.P_flashSave_s3->checkVlue))
    {
        AllStatus_S.flashSave_s.checkVlue = FLASH_CHECK_VLUEb;
        AllStatus_S.flashSave_s.TarTemp = FIRST_SOLDERING_TEMP;
        AllStatus_S.flashSave_s.BuzOnOff = 1;
        AllStatus_S.flashSave_s.calibration_temp = 0;
        AllStatus_S.flashSave_s.PreinstallTempOnOff = 0;
        AllStatus_S.flashSave_s.PreinstallTempNum = 1;
        AllStatus_S.flashSave_s.BackgroundLightOnoff = 0;
        AllStatus_S.flashSave_s.T245PowerCompensation = 0;
        AllStatus_S.flashSave_s.DisplayPowerOnOff = 0;
        AllStatus_S.flashSave_s.SaveNum = 0;
        Drive_FlashSaveData();
    }
    else
    {
        max_index = app_getMaxOfnum(AllStatus_S.P_flashSave_s1->SaveNum,
                                    AllStatus_S.P_flashSave_s2->SaveNum,
                                    AllStatus_S.P_flashSave_s3->SaveNum);
        switch (max_index)
        {
        case 0:
            Drive_FlashRed(AllStatus_S.P_flashSave_s1);
            break;
        case 1:
            Drive_FlashRed(AllStatus_S.P_flashSave_s2);
            break;
        case 2:
            Drive_FlashRed(AllStatus_S.P_flashSave_s3);
            break;
        }
    }
}

void Drive_FlashSaveInit(void)
{
    uint32_t primask_bit;
    /* Enter critical section */
    primask_bit = __get_PRIMASK();
    __disable_irq();

    HAL_IWDG_Refresh(&IwdgHandle);

    /* Lock FLASH */
    Drive_FlashVerify();

    /* Exit critical section: restore previous priority mask */
    __set_PRIMASK(primask_bit);
}
