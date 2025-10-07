#include "main.h"

uint8_t displayMemory[ALL_SEG_NUM] = {0};

void Drive_DisplayLcd_Gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruct.Pin = HG1612_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(HG1612_CS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = HG1612_DIO_PIN | HG1612_CLK_PIN;
    HAL_GPIO_Init(HG1612_DIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(HG1612_DIO_PORT, HG1612_DIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HG1612_CLK_PORT, HG1612_CLK_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HG1612_CS_PORT, HG1612_CS_PIN, GPIO_PIN_SET);
}

static void Drive_DisplayLcd_delay(void)
{
    uint8_t i;
    for (i = 0; i < 10; i++)
    {
        __NOP();
    }
}

static void DisplayLcd_CsColse(void)
{
    HG1612_CS_UP();
    Drive_DisplayLcd_delay();
    HG1612_CLK_UP();
    HG1612_DIO_UP();
    Drive_DisplayLcd_delay();
}

static void Drive_DisplayLcd_SendCmd(uint32_t LcdCmd) // 设置命令
{
    uint8_t i;
    uint32_t temp;

    HG1612_CS_DOWM();
    Drive_DisplayLcd_delay();
    temp = LcdCmd;
    for (i = 0; i < 12; i++)
    {
        HG1612_CLK_DOWM();
        if (temp & 0x800)
            HG1612_DIO_UP();
        else
            HG1612_DIO_DOWM();
        Drive_DisplayLcd_delay();
        HG1612_CLK_UP();
        Drive_DisplayLcd_delay();
        temp = temp << 1;
    }
    DisplayLcd_CsColse();
}

void Drive_DisplayLcd_Init(void)
{
    Drive_DisplayLcd_SendCmd(ENABLE_SYS);  // 打开系统时钟
    Drive_DisplayLcd_SendCmd(SET_COMS);    // 4COMS,表示 1/3 偏置
    Drive_DisplayLcd_SendCmd(START_VIDEO); // 打开显示
}

static void Drive_DisplayLcd_sendAddr(uint32_t LcdCmd) // 发送地址
{
    uint8_t i;
    uint32_t temp;

    temp = LcdCmd;
    for (i = 0; i < 9; i++)
    {
        HG1612_CLK_DOWM();
        if (temp & 0x100)
            HG1612_DIO_UP();
        else
            HG1612_DIO_DOWM();
        Drive_DisplayLcd_delay();
        HG1612_CLK_UP();
        Drive_DisplayLcd_delay();
        temp = temp << 1;
    }
}

static void Drive_DisplayLcd_sendMessage(void) // 数据
{
    uint8_t i, j;
    uint8_t temp;

    for (j = 0; j < ALL_SEG_NUM; j++)
    {
        temp = displayMemory[j];
        for (i = 0; i < 4; i++)
        {
            HG1612_CLK_DOWM();
            if (temp & 0x01)
                HG1612_DIO_UP();
            else
                HG1612_DIO_DOWM();
            Drive_DisplayLcd_delay();
            HG1612_CLK_UP();
            Drive_DisplayLcd_delay();
            temp = temp >> 1;
        }
    }
}

void Drive_DisplayLcd_sendData_Task(void)
{
    uint32_t primask_bit;

    HG1612_CS_DOWM();
    Drive_DisplayLcd_delay();

    /* Enter critical section */
    primask_bit = __get_PRIMASK();
    __disable_irq();

    Drive_DisplayLcd_sendAddr(SEG9ADDR);

    Drive_DisplayLcd_sendMessage();

    DisplayLcd_CsColse();

    /* Exit critical section: restore previous priority mask */
    __set_PRIMASK(primask_bit);
}
