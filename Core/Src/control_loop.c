/*
 * control_loop.c
 *
 *  Created on: 20 déc. 2022
 *      Author: lucas
 */
#include "control_loop.h"


void speedControlLoop_init(h_speedControlLoop_t * controlLoop, h_zxbm_t * motor, int kp, int ki, int antiWindupThreshold, int correctionFactor){
	controlLoop->motor = motor;
	zxbm_setDutyCycle(motor, 0);

	controlLoop->kp = kp;
	controlLoop->ki = ki;
	controlLoop->antiWindupThreshold = antiWindupThreshold;

	controlLoop->correctionFactor = correctionFactor;

	controlLoop->absolutePostion = 0;
	controlLoop->sumErrorSpeed = 0;

	/*Default values*/
	controlLoop->accelStep = 2;
}

void speedControlLoop_updateSpeed(h_speedControlLoop_t * controlLoop, int setSpeed){
	controlLoop->previousCNT = controlLoop->currentCNT;
	controlLoop->currentCNT = controlLoop->encoder.getCounter();


	if(controlLoop->encoder.isCountingDown()){
		controlLoop->diffCNT = (controlLoop->previousCNT < controlLoop->currentCNT)? controlLoop->previousCNT + (65535 - controlLoop->currentCNT) : controlLoop->previousCNT - controlLoop->currentCNT;
		if(controlLoop->diffCNT < 200 || controlLoop->diffCNT > -200)
			controlLoop->absolutePostion = controlLoop->absolutePostion + controlLoop->diffCNT*((controlLoop->motor->isReversed == 0)? -1 : 1);
	}else{
		controlLoop->diffCNT = (controlLoop->currentCNT < controlLoop->previousCNT)? controlLoop->currentCNT + (65535 - controlLoop->previousCNT) : controlLoop->currentCNT - controlLoop->previousCNT;
		if(controlLoop->diffCNT < 200 || controlLoop->diffCNT > -200)
			controlLoop->absolutePostion = controlLoop->absolutePostion + controlLoop->diffCNT*((controlLoop->motor->isReversed == 0)? 1 : -1);
	}

	controlLoop->currentSpeed = (controlLoop->diffCNT)*(1000 + controlLoop->correctionFactor);
	controlLoop->errorSpeed = 1000*setSpeed - controlLoop->currentSpeed;

	if (abs(controlLoop->errorSpeed) < controlLoop->antiWindupThreshold*1000)
		controlLoop->sumErrorSpeed += controlLoop->errorSpeed;

	controlLoop->newDutyCycle = controlLoop->kp*controlLoop->errorSpeed + controlLoop->sumErrorSpeed/controlLoop->ki;

	if(controlLoop->currentDutyCycle < controlLoop->newDutyCycle)
		controlLoop->currentDutyCycle += (1000 + 2*controlLoop->correctionFactor)*controlLoop->accelStep;
	else
		controlLoop->currentDutyCycle = controlLoop->newDutyCycle;

	zxbm_setDutyCycle(controlLoop->motor, (controlLoop->currentDutyCycle)/1000);
}
