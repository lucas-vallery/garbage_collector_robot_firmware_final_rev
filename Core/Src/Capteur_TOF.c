/*
 * Capteur_TOF.c
 *
 *  Created on: Dec 12, 2022
 *      Author: Diego
 */

#include "main.h"
#include "Capteur_TOF.h"
#include "i2c.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;		// UART de debug


int i=0;

VL53L0X_Error rightError;
VL53L0X_Error rightStatus;

VL53L0X_Error leftError;
VL53L0X_Error leftStatus;



extern VL53L0X_RangingMeasurementData_t rightTofStruct;
extern VL53L0X_Dev_t  rightTofDev;
extern VL53L0X_RangingMeasurementData_t leftTofStruct;
extern VL53L0X_Dev_t  leftTofDev;

//extern uint8_t Message[64];
//extern uint8_t MessageLen;
extern uint32_t rightRefSpadCount;
extern uint8_t rightIsApertureSpads;
extern uint8_t rightVhvSettings;
extern uint8_t rightPhaseCal;

extern uint32_t leftRefSpadCount;
extern uint8_t leftIsApertureSpads;
extern uint8_t leftVhvSettings;
extern uint8_t leftPhaseCal;

/*
void GPIO_write(GPIO_TypeDef * gpio_port,uint16_t gpio_pin,GPIO_PinState gpio_PinState ){
	HAL_GPIO_WritePin(gpio_port,gpio_pin,gpio_PinState);
}
*/

//void TOF1_Init(void){
//	Dev->I2cHandle = &hi2c1;
//	Dev->I2cDevAddr = 0x52;
//
//	GPIO_write(X_SHUT_TOF1_GPIO_Port, X_SHUT_TOF1_Pin, GPIO_PIN_SET); // Enable XSHUT
//	HAL_Delay(20);
//
//	VL53L0X_WaitDeviceBooted( Dev );
//	VL53L0X_DataInit( Dev );
//	VL53L0X_StaticInit( Dev );
//	VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
//	VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
//	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
//	TOF_calib();
//}

void rightTofInitInterrupt(void){
	rightTofDev.I2cHandle = &hi2c1;
	rightTofDev.I2cDevAddr = 0x52;
	rightTofDev.comms_speed_khz = 400;
	rightTofDev.comms_type = 1;

	HAL_GPIO_WritePin(X_SHUT_RIGHT_GPIO_Port, X_SHUT_RIGHT_Pin, GPIO_PIN_SET);
	//GPIO_write(X_SHUT_RIGHT_GPIO_Port, X_SHUT_RIGHT_Pin, GPIO_PIN_SET); // Enable XSHUT
	HAL_Delay(20);

	rightError=VL53L0X_DataInit(&rightTofDev);
	printf("error DataInit =: %d\r\n",rightError);
	rightError=VL53L0X_StaticInit(&rightTofDev);
	printf("error StaticInit =: %d\r\n",rightError);

	rightError=VL53L0X_PerformRefSpadManagement(&rightTofDev, &rightRefSpadCount, &rightIsApertureSpads);
	printf("error performRef =: %d\r\n",rightError);
	rightError=VL53L0X_PerformRefCalibration(&rightTofDev, &rightVhvSettings, &rightPhaseCal);
	printf("error performCalibration =: %d\r\n",rightError);


	rightError=VL53L0X_SetDeviceMode(&rightTofDev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	printf("error setDeviceMode=: %d\r\n",rightError);

	rightError=vl53l0x_Right_Range_Profiles(&rightTofDev);
	printf("error RangeProfile=: %d\r\n",rightError);
	rightError=VL53L0X_SetGpioConfig(&rightTofDev,0,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, VL53L0X_INTERRUPTPOLARITY_HIGH);
	printf("error SetGPIOConfig=: %d\r\n",rightError);
	VL53L0X_ClearInterruptMask(&rightTofDev,0);
	printf("error ClearInterruptMaskt=: %d\r\n",rightError);

	VL53L0X_SetDeviceAddress(&rightTofDev, 0x53);
	rightTofDev.I2cDevAddr = 0x53;
}


void leftTofInitInterrupt(void){
	leftTofDev.I2cHandle = &hi2c1;
	leftTofDev.I2cDevAddr = 0x52;
	leftTofDev.comms_speed_khz = 400;
	leftTofDev.comms_type = 1;

	HAL_GPIO_WritePin(X_SHUT_LEFT_GPIO_Port, X_SHUT_LEFT_Pin, GPIO_PIN_SET);
	//GPIO_write(X_SHUT_RIGHT_GPIO_Port, X_SHUT_RIGHT_Pin, GPIO_PIN_SET); // Enable XSHUT
	HAL_Delay(20);


	leftError=VL53L0X_DataInit(&leftTofDev);
	printf("error DataInit =: %d\r\n",leftError);
	leftError=VL53L0X_StaticInit(&leftTofDev);
	printf("error StaticInit =: %d\r\n",leftError);
	leftError=VL53L0X_PerformRefSpadManagement(&leftTofDev, &leftRefSpadCount, &leftIsApertureSpads);
	printf("error performRef =: %d\r\n",leftError);
	leftError=VL53L0X_PerformRefCalibration(&leftTofDev, &leftVhvSettings, &leftPhaseCal);
	printf("error performCalibration =: %d\r\n",leftError);
	leftError=VL53L0X_SetDeviceMode(&leftTofDev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	printf("error setDeviceMode=: %d\r\n",leftError);
	leftError=vl53l0x_Left_Range_Profiles(&leftTofDev);
	printf("error RangeProfile=: %d\r\n",leftError);
	leftError=VL53L0X_SetGpioConfig(&leftTofDev,0,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, VL53L0X_INTERRUPTPOLARITY_HIGH);
	printf("error SetGPIOConfig=: %d\r\n",leftError);
	VL53L0X_ClearInterruptMask(&leftTofDev,0);
	printf("error ClearInterruptMaskt=: %d\r\n",leftError);

	VL53L0X_SetDeviceAddress(&leftTofDev, 0x54);
	leftTofDev.I2cDevAddr = 0x54;
}
//void TOF_calib(void){
//	// Enable/Disable Sigma and Signal check
//	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
//	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
//	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
//	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
//	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
//	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
//}

//void tof_measure_interrupt(void){
//	VL53L0X_PerformSingleRangingMeasurement(Dev,&RangingData);
//}

//void TOF_Meausure (void){
//	  VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
//
//			  if(RangingData.RangeStatus == 0)
//			  {
//				  MessageLen = sprintf((char*)Message, "Sensor %d : %i mm\n\r",i, RangingData.RangeMilliMeter);
//				  HAL_UART_Transmit(&huart1, Message, MessageLen, 100);
//				  HAL_Delay(500);
//			  }
//			  i=1-i;
//}
/*
void TOF_alterne_measure(void){

	// measure TOF1
	TOF1_Init();												// enable tof 1
	TOF_Meausure();												// measure with tof 1
	GPIO_write(X_SHUT_TOF1_GPIO_Port,X_SHUT_TOF1_Pin,0);		// disabe tof 1
	HAL_Delay(2000);

	// measure TOF 2
	//TOF2_Init();												// enable tof 2
	TOF_Meausure();												// measure with tof 2
	GPIO_write(X_SHUT_TOF2_GPIO_Port,X_SHUT_TOF2_Pin,0);		// disabe tof 2
	HAL_Delay(2000);
}
 */

uint16_t vl53l0x_PerformRangingMeasurement(VL53L0X_DEV Dev,VL53L0X_RangingMeasurementData_t *VL53L0X_RangingMeasurementData){

	VL53L0X_GetRangingMeasurementData(Dev,VL53L0X_RangingMeasurementData);
	VL53L0X_ClearInterruptMask(Dev, 0);

	return VL53L0X_RangingMeasurementData->RangeMilliMeter;
}

VL53L0X_Error vl53l0x_Right_Range_Profiles(VL53L0X_DEV Dev)
{
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

	switch(RANGE_PROFILE_SELECTED)
	{
	case LONG_RANGE:
		signalLimit = (FixPoint1616_t)(0.1*65536);
		sigmaLimit = (FixPoint1616_t)(60*65536);
		timingBudget = 33000;
		preRangeVcselPeriod = 18;
		finalRangeVcselPeriod = 14;
		break;
	case HIGH_ACCURACY:
		signalLimit = (FixPoint1616_t)(0.25*65536);
		sigmaLimit = (FixPoint1616_t)(18*65536);
		timingBudget = 200000;
		preRangeVcselPeriod = 14;
		finalRangeVcselPeriod = 10;
		break;
	case HIGH_SPEED:
		signalLimit = (FixPoint1616_t)(0.25*65536);
		sigmaLimit = (FixPoint1616_t)(32*65536);
		timingBudget = 20000;
		preRangeVcselPeriod = 14;
		finalRangeVcselPeriod = 10;
		break;
	default:
		printf("Not Supported");
	}

	VL53L0X_SetLimitCheckValue(&rightTofDev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
	VL53L0X_SetLimitCheckValue(&rightTofDev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&rightTofDev,timingBudget);
	VL53L0X_SetVcselPulsePeriod(&rightTofDev,VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
	VL53L0X_SetVcselPulsePeriod(&rightTofDev,VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);

	return rightStatus;
}

VL53L0X_Error vl53l0x_Left_Range_Profiles(VL53L0X_DEV Dev)
{
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

	switch(RANGE_PROFILE_SELECTED)
	{
	case LONG_RANGE:
		signalLimit = (FixPoint1616_t)(0.1*65536);
		sigmaLimit = (FixPoint1616_t)(60*65536);
		timingBudget = 33000;
		preRangeVcselPeriod = 18;
		finalRangeVcselPeriod = 14;
		break;
	case HIGH_ACCURACY:
		signalLimit = (FixPoint1616_t)(0.25*65536);
		sigmaLimit = (FixPoint1616_t)(18*65536);
		timingBudget = 200000;
		preRangeVcselPeriod = 14;
		finalRangeVcselPeriod = 10;
		break;
	case HIGH_SPEED:
		signalLimit = (FixPoint1616_t)(0.25*65536);
		sigmaLimit = (FixPoint1616_t)(32*65536);
		timingBudget = 20000;
		preRangeVcselPeriod = 14;
		finalRangeVcselPeriod = 10;
		break;
	default:
		printf("Not Supported");
	}

	VL53L0X_SetLimitCheckValue(&leftTofDev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
	VL53L0X_SetLimitCheckValue(&leftTofDev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&leftTofDev,timingBudget);
	VL53L0X_SetVcselPulsePeriod(&leftTofDev,VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
	VL53L0X_SetVcselPulsePeriod(&leftTofDev,VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);

	return leftStatus;
}
