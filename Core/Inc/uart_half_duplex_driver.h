/*
 * uart_half_duplex_driver.h
 *
 *  Created on: Oct 27, 2022
 *      Author: lucas
 */

#pragma once

#include "usart.h"

#define USART_PORT huart2

int uart_half_duplex_transmit(uint8_t *pData, uint16_t size, uint32_t timeout);

int uart_half_duplex_receive(uint8_t *pData, uint16_t size, uint32_t timeout);
