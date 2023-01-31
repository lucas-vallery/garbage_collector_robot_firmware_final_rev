/*
 * control_loop.h
 *
 *  Created on: 20 déc. 2022
 *      Author: lucas
 */

#ifndef INC_CONTROL_LOOP_H_
#define INC_CONTROL_LOOP_H_

#include "zxbm_driver.h"
#include "drv_timer.h"
#include <stdlib.h>
#include <stdio.h>


typedef uint32_t (* drv_get_encoder_counter_t)();
typedef int (* drv_encoder_is_counting_down_t)();

typedef struct {
	drv_get_encoder_counter_t 		getCounter;
	drv_encoder_is_counting_down_t 	isCountingDown;
}h_encoder_t;

typedef struct {
	h_zxbm_t * motor;

	h_encoder_t encoder;

	int kp, ki;
	int currentCNT, previousCNT, diffCNT;
	int currentSpeed, previousSpeed, errorSpeed, sumErrorSpeed;;
	int currentDutyCycle, newDutyCycle;

	int absolutePostion;

	int antiWindupThreshold;

	int correctionFactor;

	uint32_t accelStep;
} h_speedControlLoop_t;

void speedControlLoop_init(h_speedControlLoop_t * controlLoop, h_zxbm_t * motor, int kp, int ki, int antiWindupThreshold, int correctionFactor);
void speedControlLoop_updateSpeed(h_speedControlLoop_t * controlLoop, int setSpeed);


#endif /* INC_CONTROL_LOOP_H_ */
