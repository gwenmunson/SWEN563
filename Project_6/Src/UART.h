#ifndef __STM32L476G_DISCOVERY_UART_H
#define __STM32L476G_DISCOVERY_UART_H

//#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"

#define BufferSize 32


void UART_Init(void);//UART_HandleTypeDef *huart2);
void vPrintString(char *message);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* __STM32L476G_DISCOVERY_UART_H */
