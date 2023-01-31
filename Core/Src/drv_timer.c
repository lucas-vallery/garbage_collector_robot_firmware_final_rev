/*
 * drv_timer.c
 *
 *  Created on: Dec 1, 2022
 *      Author: lucas
 */
#include "drv_timer.h"

void startRightPWM(void){
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
}
void startRightPWMN(void){
	HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
}
void stopRightPWM(void){
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
}
void stopRightPWMN(void){
	HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1);
}
void setRightDutyCycle(int dutyCycle){
	uint32_t timerPeriod = __HAL_TIM_GET_AUTORELOAD(&htim15);

	uint32_t timerCompare = (dutyCycle/100.0) * timerPeriod;
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1,(uint32_t) timerCompare);
}
uint32_t getRightEncoderCounter(){
	return __HAL_TIM_GET_COUNTER(&htim1);
}
int isRightEncoderCountingDown(){
	return __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);
}


void startLeftPWM(void){
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}
void startLeftPWMN(void){
	HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
}
void stopLeftPWM(void){
	HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
}
void stopLeftPWMN(void){
	HAL_TIMEx_PWMN_Stop(&htim16, TIM_CHANNEL_1);
}
void setLeftDutyCycle(int dutyCycle){
	uint32_t timerPeriod = __HAL_TIM_GET_AUTORELOAD(&htim16);

	uint32_t timerCompare = (dutyCycle/100.0) * timerPeriod;
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,(uint32_t) timerCompare);
}
uint32_t getLeftEncoderCounter(){
	return __HAL_TIM_GET_COUNTER(&htim3);
}
int isLeftEncoderCountingDown(){
	return __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
}

