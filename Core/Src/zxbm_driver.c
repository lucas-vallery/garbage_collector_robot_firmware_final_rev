/*
 * zxbm.c
 *
 *  Created on: Dec 1, 2022
 *      Author: lucas
 */
#include "zxbm_driver.h"

void zxbm_setDutyCycle(h_zxbm_t* driver, int dutyCycle){
	dutyCycle = (dutyCycle > 99)? 99 : dutyCycle;
	dutyCycle = (dutyCycle <  0)?  0 : dutyCycle;

	driver->timer.setDutyCycle(dutyCycle);
}

void zxbm_run(h_zxbm_t* driver, h_zxbm_direction_t dir){
	h_zxbm_direction_t newDir;
	newDir = (driver->isReversed) ^ dir;

	switch(newDir){
	case zxbm_backward :
		driver->timer.startPWM();
		driver->timer.stopPWMN();
		break;
	case zxbm_forward :
		driver->timer.stopPWM();
		driver->timer.startPWMN();
		break;
	/*
	case zxbm_hold :
		driver->timer.startPWM();
		driver->timer.startPWMN();
		driver->timer.setDutyCycle(99);
		break;
		*/
	default :
		driver->timer.stopPWM();
		driver->timer.stopPWMN();
	}
}
