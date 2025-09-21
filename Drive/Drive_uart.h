#ifndef __DRIVE_UART_H__
#define __DRIVE_UART_H__

/* 宏定义 */
#define		DEBUG_UART_Rx_GPIO_CLK_ENABLE			__HAL_RCC_GPIOA_CLK_ENABLE
#define		DEBUG_UART_Rx_GPIO_PORT					GPIOA
#define		DEBUG_UART_Rx_GPIO_PIN					GPIO_PIN_13

#define		DEBUG_UART_Tx_GPIO_CLK_ENABLE			__HAL_RCC_GPIOA_CLK_ENABLE
#define		DEBUG_UART_Tx_GPIO_PORT					GPIOA
#define		DEBUG_UART_Tx_GPIO_PIN	                GPIO_PIN_14


extern void Drive_uart1_init(void);

#endif





