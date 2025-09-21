#include "main.h"

UART_HandleTypeDef UartHandle;

void Drive_uart1_init(void)
{
    /* Initialize USART */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    UartHandle.Instance = USART1;
    UartHandle.Init.BaudRate = 115200;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits = UART_STOPBITS_1;
    UartHandle.Init.Parity = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode = UART_MODE_TX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        APP_ErrorHandler();
    }
}

/**
 * @brief       UART底层初始化函数
 * @param       huart: UART句柄类型指针
 * @note        此函数会被HAL_UART_Init()调用
 *              完成时钟使能，引脚配置，中断配置z
 * @retval      无
 */
void HAL_UART_MspInit(UART_HandleTypeDef *UartHandle)
{
    if (UartHandle->Instance == USART1)
    {
        GPIO_InitTypeDef Uart_GPIO_InitConfig;

        /*USART1时钟使能*/
        DEBUG_UART_Rx_GPIO_CLK_ENABLE();
        DEBUG_UART_Tx_GPIO_CLK_ENABLE();

        Uart_GPIO_InitConfig.Pin = DEBUG_UART_Tx_GPIO_PIN;
        Uart_GPIO_InitConfig.Mode = GPIO_MODE_AF_PP;
        Uart_GPIO_InitConfig.Pull = GPIO_PULLUP;
        Uart_GPIO_InitConfig.Speed = GPIO_SPEED_FREQ_HIGH;
        Uart_GPIO_InitConfig.Alternate = GPIO_AF1_USART1;
        HAL_GPIO_Init(DEBUG_UART_Tx_GPIO_PORT, &Uart_GPIO_InitConfig);

        Uart_GPIO_InitConfig.Pin = DEBUG_UART_Rx_GPIO_PIN;
        Uart_GPIO_InitConfig.Mode = GPIO_MODE_AF_PP;
        Uart_GPIO_InitConfig.Pull = GPIO_PULLUP;
        Uart_GPIO_InitConfig.Speed = GPIO_SPEED_FREQ_HIGH;
        Uart_GPIO_InitConfig.Alternate = GPIO_AF1_USART1;
        HAL_GPIO_Init(DEBUG_UART_Rx_GPIO_PORT, &Uart_GPIO_InitConfig);

        //HAL_NVIC_SetPriority(USART1_IRQn, 0, 1); /* 抢占优先级0，子优先级1 */
        //HAL_NVIC_EnableIRQ(USART1_IRQn);         /* 使能USART1中断通道 */
    }
}

/**
 * @brief  USART错误回调执行函数，输出错误代码
 * @param  无
 * @retval 无
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC); // 清除发送完成标志
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
    __HAL_UART_CLEAR_IDLEFLAG(huart); // 清除空闲中断标志
    __HAL_UART_CLEAR_FEFLAG(huart);   // 清除帧错误标志
    __HAL_UART_CLEAR_NEFLAG(huart);   // 清除噪声错误标志
    __HAL_UART_CLEAR_OREFLAG(huart);  // 清除溢出错误标志
}

void USART1_IRQHandler(void)
{
    // HAL_UART_IRQHandler(&UartHandle); // 处理USART1中断请求
    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET) // 检查是否接收到数据
    {
        __NOP();
    }
    else
    {
        HAL_UART_ErrorCallback(&UartHandle); // 处理错误
    }
    __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_RXNE);
}

/**
 * @brief  USART发送回调执行函数
 * @param  无
 * @retval 无
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    __HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_TC); // 清除发送完成标志
    __HAL_UART_CLEAR_IDLEFLAG(UartHandle);           // 清除空闲中断标志
    __HAL_UART_CLEAR_FEFLAG(UartHandle);             // 清除帧错误标志
    __HAL_UART_CLEAR_NEFLAG(UartHandle);             // 清除噪声错误标志
    __HAL_UART_CLEAR_OREFLAG(UartHandle);            // 清除溢出错误标志
}


/**
 * @brief  串口发送函数
 * @param  data 要发送的数据数组
 * @param  length 数据长度
 * @retval 无
 */
uint8_t UART_SendData(uint8_t *data, uint8_t length)
{
    /* if (HAL_UART_Transmit(&UartHandle, data, length, 50) != HAL_OK)
    {
        return 0;
    } */
    for (uint8_t i = 0; i < length; i++)
    {
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_TC);
        USART1->DR = *(data++);
        while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TC) == RESET)
            ;
    }
    return 1;
}

/* int fputc(int ch, FILE *f)
{
    // HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1);
    USART1->DR = ch; // 发送数据
    while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TC) == RESET)
        ;
    __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_TC); // 清除发送完成标志
    return ch;
} */
