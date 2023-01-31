/*
 * robot.c
 *
 *  Created on: 23 déc. 2022
 *      Author: lucas
 */

#include "robot.h"

int time = 100;

static void robot_computePolarFromCartesianCoordinates(h_robot_t * robot, int * cartesianCoordinates, int * polarCoordinates){
	float distance, angle;

	distance = sqrt(pow(cartesianCoordinates[0] - robot->x, 2) + pow(cartesianCoordinates[1] - robot->y,2));

	angle = atanf(((float) (cartesianCoordinates[1] - robot->y))/((float) (cartesianCoordinates[0] - robot->x)));
	if(cartesianCoordinates[0] - robot->x < 0){
		angle += PI;
	}

	polarCoordinates[0] = (int) ceilf(distance);
	polarCoordinates[1] = ((int) ceilf(angle * RAD_TO_DEG_RATIO)) - robot->w;

	printf("Polar coordinates : %d\t %d\r\n", polarCoordinates[0], polarCoordinates[1]);
}

void robot_initStruct(h_robot_t * robot, h_speedControlLoop_t * leftMotor, h_speedControlLoop_t * rightMotor){
	robot->leftMotor 		= leftMotor;
	robot->rightMotor 		= rightMotor;

	robot->x				= 0;//75;
	robot->y				= 0;//82;
	robot->w				= 0; //Should be 45

	robot->isRobotMoving 	= 0;

	robot->leftSpeedCommand 	= 0;
	robot->rightSpeedCommand 	= 0;

	robot->oldTickLeft 	= 0;
	robot->oldTickRight = 0;

	robot->leftBorderDetectedFlag = 0;
	robot->rightBorderDetectedFlag = 0;

	robot->virtualBorderArrayIndex = 0;

	robot->waitForStart = 1;

	robot->virtualBorderSensor.xOffset = 140;
	robot->virtualBorderSensor.yOffset = 0;
	robot->virtualBorderDetectionFlag = 0;
	robot->virtualBorderDetectionFlagOld = 0;

	robot->state = search;
}

void robot_goToPosition(h_robot_t * robot, int d){
	int ticks;
	float distance, angle;
	ticks = (int) ceilf(MM_TO_TICK_RATIO * abs(d));

	robot->leftMotor->sumErrorSpeed = 0;
	robot->rightMotor->sumErrorSpeed = 0;

	robot->leftMotor->absolutePostion = 0;
	robot->rightMotor->absolutePostion = 0;

	robot->isRobotMoving = 1;

	if(d > 0){
		zxbm_run(robot->leftMotor->motor, zxbm_forward);
		zxbm_run(robot->rightMotor->motor, zxbm_forward);
	}else {
		zxbm_run(robot->leftMotor->motor, zxbm_backward);
		zxbm_run(robot->rightMotor->motor, zxbm_backward);
	}
	while(abs(robot->leftMotor->absolutePostion + robot->rightMotor->absolutePostion)/2 < abs(ticks)){
		speedControlLoop_updateSpeed(robot->rightMotor, LINEAR_SPEED);
		speedControlLoop_updateSpeed(robot->leftMotor, LINEAR_SPEED);
		vTaskDelay(6);
	}
	speedControlLoop_updateSpeed(robot->rightMotor, NO_SPEED);
	speedControlLoop_updateSpeed(robot->leftMotor, NO_SPEED);

	distance = ((float)(robot->rightMotor->absolutePostion + robot->leftMotor->absolutePostion))/(2 * MM_TO_TICK_RATIO);
	angle = ((float)(robot->rightMotor->absolutePostion - robot->leftMotor->absolutePostion))/(2*DEG_TO_TICK_RATIO);
	robot->w = robot->w + (int) ceilf(angle);

	robot->x += (int) ceilf(distance * cosf(robot->w * DEG_TO_RAD_RATIO));
	robot->y += (int) ceilf(distance * sinf(robot->w * DEG_TO_RAD_RATIO));

	robot->isRobotMoving = 0;
}

/*
void robot_goToAngle(h_robot_t * robot, int w){
	int ticks;
	float angle;
	ticks = (int) ceilf(2 * DEG_TO_TICK_RATIO * abs(w));

	robot->leftMotor->sumErrorSpeed = 0;
	robot->rightMotor->sumErrorSpeed = 0;

	robot->leftMotor->absolutePostion = 0;
	robot->rightMotor->absolutePostion = 0;

	robot->isRobotMoving = 1;

	if(w > 0){
		zxbm_run(robot->leftMotor->motor, zxbm_backward);
		zxbm_run(robot->rightMotor->motor, zxbm_forward);
	}else{
		zxbm_run(robot->leftMotor->motor, zxbm_forward);
		zxbm_run(robot->rightMotor->motor, zxbm_backward);
	}

	while(abs(robot->rightMotor->absolutePostion - robot->leftMotor->absolutePostion) < abs(ticks)){
		speedControlLoop_updateSpeed(robot->rightMotor, ROTATION_SPEED);
		speedControlLoop_updateSpeed(robot->leftMotor, ROTATION_SPEED);
		vTaskDelay(6);
	}
	speedControlLoop_updateSpeed(robot->rightMotor, NO_SPEED);
	speedControlLoop_updateSpeed(robot->leftMotor, NO_SPEED);

	angle = ((float)(robot->rightMotor->absolutePostion - robot->leftMotor->absolutePostion))/(2*DEG_TO_TICK_RATIO);
	robot->w += (int) ceilf(angle);

	robot->isRobotMoving = 0;
}
 */

void robot_goToCoordinates(h_robot_t * robot, int * cartesianCoordinates){
	int polarCoordinates[2];

	robot_computePolarFromCartesianCoordinates(robot, cartesianCoordinates, (int*) polarCoordinates);

	robot_goToAngle(robot,  polarCoordinates[1]);
	robot_goToPosition(robot, polarCoordinates[0]);

	printf("Angle : %d\t Position : [%d, %d]\r\n", robot->w, robot->x, robot->y);
}








void robot_updateOdometry(h_robot_t * robot){
	int leftDeltaTick, rightDeltaTick;
	float deltaAngle, deltaDistance;

	leftDeltaTick = robot->leftMotor->absolutePostion - robot->oldTickLeft;
	robot->oldTickLeft = robot->leftMotor->absolutePostion;

	rightDeltaTick = robot->rightMotor->absolutePostion - robot->oldTickRight;
	robot->oldTickRight = robot->rightMotor->absolutePostion;

	deltaAngle = ((float)(rightDeltaTick - leftDeltaTick))/(2*DEG_TO_TICK_RATIO);
	robot->w += (int) (ceilf(deltaAngle/ANGLE_CALIB_RATIO));//));

	deltaDistance = ((float)(rightDeltaTick + leftDeltaTick))/(2 * MM_TO_TICK_RATIO);
	robot->x += (int) ceilf(deltaDistance * cosf(robot->w * DEG_TO_RAD_RATIO));
	robot->y += (int) ceilf(deltaDistance * sinf(robot->w * DEG_TO_RAD_RATIO));

}

void robot_goToAngle(h_robot_t * robot, int w){
	//w *= ANGLE_CALIB_RATIO;
	int initialAngle = robot->w;
	if(w > 0){
		zxbm_run(robot->leftMotor->motor, zxbm_backward);
		zxbm_run(robot->rightMotor->motor, zxbm_forward);

		while((robot->w <  w + initialAngle) && (robot->leftDist > 1200)  && (robot->rightDist >1200)){
			robot->leftSpeedCommand = ROTATION_SPEED;
			robot->rightSpeedCommand = ROTATION_SPEED;
		}
	}else{
		zxbm_run(robot->leftMotor->motor, zxbm_forward);
		zxbm_run(robot->rightMotor->motor, zxbm_backward);

		while(robot->w >  w + initialAngle){
			robot->leftSpeedCommand = ROTATION_SPEED;
			robot->rightSpeedCommand = ROTATION_SPEED;
		}
	}

	robot->leftSpeedCommand = NO_SPEED;
	robot->rightSpeedCommand = NO_SPEED;

	robot->isRobotMoving = 0;
}

void robot_stop(h_robot_t * robot){
	robot->leftSpeedCommand = NO_SPEED;
	robot->rightSpeedCommand = NO_SPEED;

	robot->isRobotMoving = 0;
}

void robot_borderAvoidance(h_robot_t * robot){
	if(robot->leftBorderDetectedFlag){
		robot_goToAngle(robot, -90);
	}else if (robot->rightBorderDetectedFlag){
		robot_goToAngle(robot, +90);
	}

	/*
	if(robot->virtualBorderDetectionFlag){
		printf("virtual border hit\r\n");
		robot_goToAngle(robot, 180);
		robot->virtualBorderDetectionFlag = 0;
	}
	*/
}

void robot_addVirtualBorder(h_robot_t * robot, h_virtualBorderType_t type, h_virtualBorderSide_t side, int offset){
	if(robot->virtualBorderArrayIndex < NUMBER_OF_VIRTUAL_BORDER_SUPPORTED){
		(robot->virtualBorderArray[robot->virtualBorderArrayIndex]).type 	= type;
		(robot->virtualBorderArray[robot->virtualBorderArrayIndex]).offset 	= offset;
		(robot->virtualBorderArray[robot->virtualBorderArrayIndex]).side	= side;
		robot->virtualBorderArrayIndex ++;
	}
}

static int robot_virtualBorderSensor(int x_sensor, int y_sensor, h_virtualBorder_t border){
	switch (border.type){
	case xType:
		if(border.side*(border.offset - x_sensor) < 0){
			return 1;
		}
		return 0;
		break;
	case yType:
		return 0;

	default:
		return 0;
		break;
	}
}

void robot_virtualBorderAvoidance(h_robot_t * robot){
	int xSensor, ySensor;

	time ++;

	xSensor = robot->x + robot->virtualBorderSensor.xOffset*cosf(robot->w);
	ySensor = robot->y + robot->virtualBorderSensor.yOffset*sinf(robot->w);

	//printf("%d\t %d\t %d\r\n", xSensor, ySensor, time);
	for(int i = 0; i < robot->virtualBorderArrayIndex; i++){
		if(robot_virtualBorderSensor(xSensor, ySensor, robot->virtualBorderArray[i]) && time > 15){
			time = 0;
			robot->virtualBorderDetectionFlag = 1;
			break;
		}
	}
}

















