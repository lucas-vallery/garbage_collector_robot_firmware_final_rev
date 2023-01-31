/*
 * drv_timer.h
 *
 *  Created on: Dec 1, 2022
 *      Author: lucas
 */

#ifndef INC_DRV_TIMER_H_
#define INC_DRV_TIMER_H_

#include "tim.h"
#include "stm32g0xx_hal.h"


void startRightPWM(void);
void startRightPWMN(void);
void stopRightPWM(void);
void stopRightPWMN(void);
void setRightDutyCycle(int dutyCycle);
uint32_t getRightEncoderCounter();
int isRightEncoderCountingDown();

void startLeftPWM(void);
void startLeftPWMN(void);
void stopLeftPWM(void);
void stopLeftPWMN(void);
void setLeftDutyCycle(int dutyCycle);
uint32_t getLeftEncoderCounter();
int isLeftEncoderCountingDown();

#endif /* INC_DRV_TIMER_H_ */
