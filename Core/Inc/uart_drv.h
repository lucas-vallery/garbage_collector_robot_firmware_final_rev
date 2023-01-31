/*
 * uart_drv.h
 *
 *  Created on: 28 nov. 2022
 *      Author: lucas
 */

#ifndef UART_DRIVER_UART_DRV_H_
#define UART_DRIVER_UART_DRV_H_

#include "shell.h"
#include "usart.h"
#include "main.h"
#include "cmsis_os.h"

void drv_uart_init();
uint8_t drv_uart_receive(char * pData, uint16_t size);
uint8_t drv_uart_transmit(char * pData, uint16_t size);


#endif /* UART_DRIVER_UART_DRV_H_ */
