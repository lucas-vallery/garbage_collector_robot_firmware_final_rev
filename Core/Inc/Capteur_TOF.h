/*
 * Capteur_TOF.h
 *
 *  Created on: Dec 12, 2022
 *      Author: Diego
 */

#ifndef INC_CAPTEUR_TOF_H_
#define INC_CAPTEUR_TOF_H_

#include "main.h"
#include "stm32g0xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include <vl53l0x_api.h>


#define RANGE_PROFILE_SELECTED 	HIGH_ACCURACY
#define LONG_RANGE 				0
#define HIGH_ACCURACY			1
#define HIGH_SPEED				2

//void GPIO_write(GPIO_TypeDef * gpio_port,uint16_t gpio_pin,GPIO_PinState gpio_PinState );


void TOF1_Init(void);
void TOF_calib(void);
void TOF_Meausure (void);
void TOF_alterne_measure(void);
void rightTofInitInterrupt(void);
void tof_measure_interrupt(void);
void tof_get_distance(void);
uint16_t vl53l0x_PerformRangingMeasurement(VL53L0X_DEV Dev,VL53L0X_RangingMeasurementData_t *VL53L0X_RangingMeasurementData);
VL53L0X_Error vl53l0x_Right_Range_Profiles(VL53L0X_DEV Dev);
VL53L0X_Error vl53l0x_Left_Range_Profiles(VL53L0X_DEV Dev);
void leftTofInitInterrupt(void);

#endif /* INC_CAPTEUR_TOF_H_ */
