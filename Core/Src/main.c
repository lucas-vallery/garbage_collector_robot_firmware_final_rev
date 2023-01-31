/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "uart_drv.h"
#include "shell.h"
#include "FreeRTOS.h"
#include "zxbm_driver.h"
#include "drv_timer.h"
#include "xl320_driver.h"
#include "control_loop.h"
#include "robot.h"
#include "Capteur_TOF.h"
#include <vl53l0x_api.h>
#include "color_sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STACK_DEPTH					500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
h_shell_t g_shell;

h_zxbm_t g_rightMotor;
h_zxbm_t g_leftMotor;

h_speedControlLoop_t g_rightControlLoop;
h_speedControlLoop_t g_leftControlLoop;

h_robot_t robot;

XL320_t xl320;

h_color_sensor_t color_sensor1;

/******************************************************************************/
VL53L0X_RangingMeasurementData_t rightTofStruct;		    //C'est une structure avec des mesures.
VL53L0X_Dev_t  rightTofDev; 							//C'est la structure de l'appareil. 						//C'est un pointeur vers la structure d'avant.

VL53L0X_RangingMeasurementData_t leftTofStruct;		    //C'est une structure avec des mesures.
VL53L0X_Dev_t  leftTofDev; 							//C'est la structure de l'appareil. 						//C'est un pointeur vers la structure d'avant.


//uint8_t Message[64];
//uint8_t MessageLen;

// VL53L0X initialization stuff
// besoin de quelques variables d'assistance pour l'initialisation
uint32_t rightRefSpadCount;
uint8_t rightIsApertureSpads;
uint8_t rightVhvSettings;
uint8_t rightPhaseCal;

uint32_t leftRefSpadCount;
uint8_t leftIsApertureSpads;
uint8_t leftVhvSettings;
uint8_t leftPhaseCal;

//uint16_t distance=0;
VL53L0X_Error rightError2;
VL53L0X_Error leftError2;



int leftDist =0, rightDist = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

int __io_getchar(void){
	uint8_t ch = 0;

	/* Clear the Overrun flag just before receiving the first character */
	__HAL_UART_CLEAR_OREFLAG(&huart1);

	/* Wait for reception of a character on the USART RX line and echo this
	 * character on console */
	HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

static void tofTask(void * unused){
	while(1){
		robot.leftDist = vl53l0x_PerformRangingMeasurement(&rightTofDev,&rightTofStruct);
		robot.rightDist = vl53l0x_PerformRangingMeasurement(&leftTofDev,&leftTofStruct);

		//printf("%d\t %d\r\n", vl53l0x_PerformRangingMeasurement(&rightTofDev,&rightTofStruct), vl53l0x_PerformRangingMeasurement(&leftTofDev,&leftTofStruct));
		vTaskDelay(5);
	}
}

static void syncTask(void * unused){

	xl320_init(&xl320, 1, BR_1M);
	xl320_setSpeed(&xl320, 20);
	xl320_torqueEnable(&xl320);
	xl320_setGoalPosition(&xl320, 69);

	while(robot.waitForStart){
		robot_stop(&robot);
		vTaskDelay(2000);
	}


	while(1){

		switch (robot.state){
		case search :

			/*
			zxbm_run(robot.leftMotor->motor, zxbm_backward);
			zxbm_run(robot.rightMotor->motor, zxbm_forward);
			while(leftDist > 1200 || rightDist > 1200){


				leftDist = vl53l0x_PerformRangingMeasurement(&rightTofDev,&rightTofStruct);
				rightDist = vl53l0x_PerformRangingMeasurement(&leftTofDev,&leftTofStruct);
			}

			 */

			zxbm_run(robot.leftMotor->motor, zxbm_forward);
			zxbm_run(robot.rightMotor->motor, zxbm_forward);

			robot.leftSpeedCommand = 8;
			robot.rightSpeedCommand = 8;
			if( robot.leftDist < 1200 || robot.rightDist < 1200){
				//robot_stop(&robot);
				robot.state = targetLocked;
			}
			break;

		case targetLocked :
			if(robot.leftDist > 230 && robot.rightDist > 230 && robot.leftDist < 1200 && robot.rightDist < 1200){
				if(robot.leftDist - robot.rightDist > 20){
					zxbm_run(robot.leftMotor->motor, zxbm_forward);
					zxbm_run(robot.rightMotor->motor, zxbm_forward);

					robot.leftSpeedCommand = 5;
					robot.rightSpeedCommand = 5+3;
				}else if(robot.leftDist - robot.rightDist < -20){
					zxbm_run(robot.leftMotor->motor, zxbm_forward);
					zxbm_run(robot.rightMotor->motor, zxbm_forward);

					robot.leftSpeedCommand = 5+3;
					robot.rightSpeedCommand = 5;
				}else{

					robot.leftSpeedCommand = 5;
					robot.rightSpeedCommand = 5;
				}
			}else if(robot.leftDist < 250 && robot.rightDist < 250){
				robot.state = capture;
			}else{
				robot.state = search;
			}
			break;

		case capture :

			xl320_setGoalPosition(&xl320, 0);
			if(robot.leftDist > 50 && robot.rightDist > 50){
				if(robot.leftDist - robot.rightDist > 20){
					zxbm_run(robot.leftMotor->motor, zxbm_forward);
					zxbm_run(robot.rightMotor->motor, zxbm_forward);

					robot.leftSpeedCommand = 3;
					robot.rightSpeedCommand = 3+2;
				}else if(robot.leftDist - robot.rightDist < -20){
					zxbm_run(robot.leftMotor->motor, zxbm_forward);
					zxbm_run(robot.rightMotor->motor, zxbm_forward);

					robot.leftSpeedCommand = 3+2;
					robot.rightSpeedCommand = 3;
				}else{

					robot.leftSpeedCommand = 3;
					robot.rightSpeedCommand = 3;
				}
			}else{
				robot.state = loaded;
			}
			break;
		case loaded :
			xl320_setGoalPosition(&xl320, 69);
			vTaskDelay(500);
			robot.state = color;

		case color :
			//colorSetPhotodiodeType(&color_sensor1, GREEN);
			//colorEnable(&color_sensor1);
			//vTaskDelay(1);
			//colorDisable(&color_sensor1);
			robot.state = locked;
			break;
		case locked :
			while(robot.leftDist < 50 && robot.rightDist < 50){
				robot_stop(&robot);
				//robot.leftDist = vl53l0x_PerformRangingMeasurement(&rightTofDev,&rightTofStruct);
				//robot.rightDist = vl53l0x_PerformRangingMeasurement(&leftTofDev,&leftTofStruct);
				vTaskDelay(20);
			}
			vTaskDelay(1000);
			robot.state = search;
			break;
		default :
			robot_stop(&robot);
		}
		//robot_stop(&robot);
		robot_borderAvoidance(&robot);

		//printf("%d\r\n", robot.state);
		vTaskDelay(5);
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_10){
		robot.leftBorderDetectedFlag = 1;
		robot_stop(&robot);
	}else if(GPIO_Pin == GPIO_PIN_11){
		robot.rightBorderDetectedFlag = 1;
		robot_stop(&robot);
	}else if(GPIO_Pin == GPIO_PIN_13){
		robot.waitForStart = 0;
	}
}



void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_10){
		robot.leftBorderDetectedFlag = 0;
		robot_stop(&robot);
	}else if(GPIO_Pin == GPIO_PIN_11){
		robot.rightBorderDetectedFlag = 0;
		robot_stop(&robot);
	}


}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM15_Init();
	MX_TIM16_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_TIM7_Init();
	MX_TIM17_Init();
	MX_I2C1_Init();
	MX_USART3_UART_Init();
	MX_TIM14_Init();
	/* USER CODE BEGIN 2 */
	xl320.serial.transmit = uart_half_duplex_transmit;
	xl320.serial.receive  = uart_half_duplex_receive;
	/* CONTROL LOOP INIT BEGIN */
	g_rightControlLoop.encoder.getCounter = getRightEncoderCounter;
	g_rightControlLoop.encoder.isCountingDown = isRightEncoderCountingDown;

	g_leftControlLoop.encoder.getCounter = getLeftEncoderCounter;
	g_leftControlLoop.encoder.isCountingDown= isLeftEncoderCountingDown;

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	g_rightMotor.timer.startPWM 		= 	startRightPWM;
	g_rightMotor.timer.startPWMN 		= 	startRightPWMN;
	g_rightMotor.timer.stopPWM 			=	stopRightPWM;
	g_rightMotor.timer.stopPWMN 		= 	stopRightPWMN;
	g_rightMotor.timer.setDutyCycle 	= 	setRightDutyCycle;
	g_rightMotor.isReversed 			= 	noReversed;

	g_leftMotor.timer.startPWM			= 	startLeftPWM;
	g_leftMotor.timer.startPWMN			=	startLeftPWMN;
	g_leftMotor.timer.stopPWM			=	stopLeftPWM;
	g_leftMotor.timer.stopPWMN			=	stopLeftPWMN;
	g_leftMotor.timer.setDutyCycle		=	setLeftDutyCycle;
	g_leftMotor.isReversed 				= 	reversed;

	speedControlLoop_init(&g_rightControlLoop, &g_rightMotor, 15, 3, 80, 0);
	speedControlLoop_init(&g_leftControlLoop, &g_leftMotor, 15, 3, 80, -15);

	robot_initStruct(&robot, &g_leftControlLoop, &g_rightControlLoop);

	zxbm_run(robot.leftMotor->motor, zxbm_forward);
	zxbm_run(robot.rightMotor->motor, zxbm_forward);

	/*************************************************************/
	printf("--- TOF--- \r\n");
	//TOF 1
	HAL_GPIO_WritePin(X_SHUT_RIGHT_GPIO_Port, X_SHUT_RIGHT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(X_SHUT_LEFT_GPIO_Port, X_SHUT_LEFT_Pin, GPIO_PIN_RESET);


	leftTofInitInterrupt();
	rightTofInitInterrupt();

	leftError2=VL53L0X_StartMeasurement(&leftTofDev);
	rightError2=VL53L0X_StartMeasurement(&rightTofDev);
	printf("error start measurement : %d\r\n",rightError2);
	/************************************************************/

	setvbuf(stdin, NULL, _IONBF, 0);

	//colorSensorInit(&color_sensor1, GREEN,CENT_POUR_CENT,SENSOR_DISABLE);
	//colorHandleCalibrationSensor(&color_sensor1);

	/************************************************************/
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim17);

	//robot_addVirtualBorder(&robot, xType, inferior, 600);
	//robot_addVirtualBorder(&robot, xType, superior, 0);

	TaskHandle_t syncTaskHandler = NULL;

	if(pdTRUE != xTaskCreate(syncTask, "syncTask", STACK_DEPTH, (void*) NULL, SHELL_TASK_PRIORITY + 3, &syncTaskHandler)) {
		printf("Control loop task failed");
		while(1);
	}

	TaskHandle_t tofTaskHandler = NULL;

	if(pdTRUE != xTaskCreate(tofTask, "tofTask", STACK_DEPTH, (void*) NULL, SHELL_TASK_PRIORITY + 2, &tofTaskHandler)) {
		printf("TOF loop task failed");
		while(1);
	}

	drv_uart_init();
	g_shell.uart.receive = drv_uart_receive;
	g_shell.uart.transmit = drv_uart_transmit;
	sh_init(&g_shell);
	sh_start(&g_shell);

	vTaskStartScheduler();
	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
