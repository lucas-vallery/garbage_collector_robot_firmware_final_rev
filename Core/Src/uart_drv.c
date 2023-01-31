/*
 * uart_drv.c
 *
 *  Created on: 28 nov. 2022
 *      Author: lucas
 */
#include "uart_drv.h"

// crée le semaphore (fonction init)

SemaphoreHandle_t sem_usart1;

void drv_uart_init(){
	sem_usart1 = xSemaphoreCreateBinary();
	//xSemaphoreGive(sem_usart1);
}

// fonction callback que tu appelles dans l'interruption
// la fonction callback fait un sem give(fromISR)

uint8_t drv_uart_receive(char * pData, uint16_t size){
	if(HAL_OK == HAL_UART_Receive_IT(&huart1, (uint8_t*) pData, size))
	{
		xSemaphoreTake(sem_usart1, portMAX_DELAY);
		return 0;
	}
	return -1;
}

uint8_t drv_uart_transmit(char * pData, uint16_t size){
	if(HAL_OK == HAL_UART_Transmit(&huart1, (uint8_t*) pData, size, HAL_MAX_DELAY))
		return 0;
	return 1;
}
