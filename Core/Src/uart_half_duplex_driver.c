/*
 * uart_half_duplex_driver.c
 *
 *  Created on: Oct 27, 2022
 *      Author: lucas
 */
#include "uart_half_duplex_driver.h"

int uart_half_duplex_transmit(uint8_t *pData, uint16_t size, uint32_t timeout){
	HAL_HalfDuplex_EnableTransmitter(&USART_PORT);
	HAL_UART_Transmit(&USART_PORT, pData, size, timeout);

	return 0;
}

int uart_half_duplex_receive(uint8_t *pData, uint16_t size, uint32_t timeout) {
	HAL_HalfDuplex_EnableReceiver(&USART_PORT);
	HAL_UART_Receive(&USART_PORT, pData, size, timeout);

	return 0;
}
