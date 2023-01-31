/*
 * robot.h
 *
 *  Created on: 23 déc. 2022
 *      Author: lucas
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include "zxbm_driver.h"
#include "control_loop.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>


#define LINEAR_SPEED 		10
#define ROTATION_SPEED 		3
#define NO_SPEED			0
#define MM_TO_TICK_RATIO	4*2.585889
#define DEG_TO_TICK_RATIO	15.164450
#define RAD_TO_DEG_RATIO	57.295780
#define DEG_TO_RAD_RATIO	0.017453
#define PI					3.141593
#define ANGLE_CALIB_RATIO	1.075

#define NUMBER_OF_VIRTUAL_BORDER_SUPPORTED	4


typedef enum {
	xType,
	yType,
}h_virtualBorderType_t;

typedef enum {
	superior = -1,
	inferior = 1,
}h_virtualBorderSide_t;

typedef struct {
	int offset;
	h_virtualBorderType_t type;
	h_virtualBorderSide_t side;
}h_virtualBorder_t;

typedef struct {
	int xOffset;
	int yOffset;
}h_virtualBorderSensor_t;

typedef enum{
	targetLocked = 0,
	capture = 1,
	search = 2,
	loaded = 3,
	color = 4,
	locked = 5,
}h_robotState_t;

typedef struct {
	h_speedControlLoop_t * leftMotor;
	h_speedControlLoop_t * rightMotor;

	h_virtualBorderSensor_t virtualBorderSensor;

	h_virtualBorder_t virtualBorderArray[NUMBER_OF_VIRTUAL_BORDER_SUPPORTED];
	int virtualBorderArrayIndex;

	int x;	//in mm
	int y;	//in mm
	int w;	//in deg

	int leftSpeedCommand;
	int rightSpeedCommand;

	int oldTickLeft;
	int oldTickRight;

	int isRobotMoving;

	int leftBorderDetectedFlag;
	int rightBorderDetectedFlag;
	int virtualBorderDetectionFlag;
	int virtualBorderDetectionFlagOld;

	int waitForStart;

	int leftDist;
	int rightDist;

	h_robotState_t state;
}h_robot_t;

void robot_initStruct(h_robot_t * robot, h_speedControlLoop_t * leftMotor, h_speedControlLoop_t * rightMotor);

void robot_goToPosition(h_robot_t * robot, int d);
void robot_goToCoordinates(h_robot_t * robot, int * cartesianCoordinates);


void robot_goToAngle(h_robot_t * robot, int w);
void robot_updateOdometry(h_robot_t * robot);
void robot_stop(h_robot_t * robot);
void robot_borderAvoidance(h_robot_t * robot);
void robot_virtualBorderAvoidance(h_robot_t * robot);
void robot_addVirtualBorder(h_robot_t * robot, h_virtualBorderType_t type, h_virtualBorderSide_t side, int offset);

#endif /* INC_ROBOT_H_ */

