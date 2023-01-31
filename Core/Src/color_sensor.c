/**
 * @file Color_sensor.c / Garbage Collector project
 * @brief Source file that contain the definition and the code of objects and functions used to operate of the color sensor
 * @author Baptiste Cottu
 * @date 19/12/2022
 */

#include "main.h"
#include "color_sensor.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// extern variables a modifier pour l'integration

//extern UART_HandleTypeDef huart1;		// UART de debug
extern TIM_HandleTypeDef htim14;			//<! Timer en input capture
//extern TIM_HandleTypeDef htim3;			//<! Timer de gestion des filtres

extern h_color_sensor_t color_sensor1; 	// a inclure dans le fichier it.c aussi


// parametrage des variables timers.
#define TIMER_INPUT_CAPTURE htim2
#define TIMER_BASE_IT htim3
#define TIMER_IC_CHANNEL TIM_CHANNEL_1


#define FREQ_MAX_ACCEPTABLE 50000
#define DECHET_MEASURE 2
#define CALIBRATION_NB_VALUES 30

// mise à l'echelle pour comparaison des frequences de sortie

#define ECHELLE_VAL_HAUTE 20000
#define ECHELLE_VAL_BASSE 10000

// fin de variable a modifier pour l'intégration

#define freeRTOS 1					//<! variable qui indique si il y a freeRTOS actif


//private variables
uint32_t color_tab[2];
uint32_t freq=0;
static int temps1;
static int temps2;
static int tour=0;
static uint32_t color_scaled_values[NB_MEASURE_WANTED+(2*DECHET_MEASURE)];

//--debug
static uint32_t color_raw_values[NB_MEASURE_WANTED+(2*DECHET_MEASURE)];

static uint8_t raw_values_compteur=0;
uint32_t green_color_value=0;
uint32_t red_color_value=0;

//used for calibration
uint8_t calibration_flag=0;
uint32_t calib_tab[CALIBRATION_NB_VALUES];
int u=0;


// static function prototype
/**
 * @fn int colorSetOutputFreq_scaling(h_color_sensor_t * h_color_sensor)
 * @brief Set the scale of the output signal
 *
 * @param h_color_sensor : color_sensor structure
 * @return 0 if failed else 1
 */
static void colorSetOutputFreqScaling(h_color_sensor_t * h_color_sensor);

/**
 * @fn void colorAnalyse(void)
 * @brief This function compares the results in color_tab. It occurs after the two mesurements are done.
 *
 * @param h_color_sensor : color_sensor structure
 * @return : Printf the color of the can
 */
static void colorAnalyse(h_color_sensor_t * h_color_sensor, color_t green, color_t red);

/**
 * @fn void colorDoMeasureAgain(h_color_sensor_t * h_color_sensor)
 * @brief This function is called to do a measure again after a inoperable result
 *
 * @param h_color_sensor : color_sensor structure
 * @return : Printf the color of the can
 */
static void colorDoMeasureAgain(h_color_sensor_t * h_color_sensor);

/**
 * @fn int colorHandleCalibrationValues(h_color_sensor_t * h_color_sensor, uint32_t frequence,h_calib_buffer_structure_t * h_calib_struct)
 * @brief This function fills the field of the calibration_buffer_structure
 *
 * @param h_color_sensor : color_sensor structure
 * @param h_calib_buffer_structure : calibration structure to fill
 * @return : 1 if calibration is processing, 0 if finished
 */
static int colorHandleCalibrationValues(h_color_sensor_t * h_color_sensor, uint32_t frequence);

/**
 * @fn int colorHandleRawValues(h_color_sensor_t * h_color_sensor, uint32_t frequence)
 * @brief This function compute the scaling of frequence and out it in a tab waiting for mean computation
 *
 * @param h_color_sensor : color_sensor structure
 * @param uint32_t frequence : raw output value of the sensor
 * @return : 1 if calibration is processing, 0 if finished
 */
static int colorHandleRawValues(h_color_sensor_t * h_color_sensor, uint32_t frequence);

/**
 * @fn static void colorMeasureTask(void * pvParameters)
 * @brief Measure task. Perform a measure. Triggered by the release of a sempahore
 *
 * @param (void *)color_sensor
 * @return : 1 if calibration is processing, 0 if finished
 */
static void colorMeasureTask(void * pvParameters);

// ----- Global Fonctions -----


// --- fonctions de gestion du hardware ---

static void colorSetOutputFreqScaling(h_color_sensor_t * h_color_sensor){
	switch(h_color_sensor->ouput_scale){
	case 4:
		// 100%
		GPIO_write(color_S0_GPIO_Port,color_S0_Pin,1);
		GPIO_write(color_S1_GPIO_Port,color_S1_Pin,1);
		break;
	case 3:
		// 20%
		GPIO_write(color_S0_GPIO_Port,color_S0_Pin,1);
		GPIO_write(color_S1_GPIO_Port,color_S1_Pin,0);
		break;
	case 2:
		// 2%
		GPIO_write(color_S0_GPIO_Port,color_S0_Pin,0);
		GPIO_write(color_S1_GPIO_Port,color_S1_Pin,1);
		break;
	case 1:
		// Power_down
		GPIO_write(color_S0_GPIO_Port,color_S0_Pin,0);
		GPIO_write(color_S1_GPIO_Port,color_S1_Pin,0);
		break;
	}
}

void colorSetPhotodiodeType(h_color_sensor_t * h_color_sensor,color_sensor_color_t color){
	h_color_sensor->color=color;
	switch(color){
	case GREEN:
		//GREEN
		GPIO_write(color_S2_GPIO_Port,color_S2_Pin,1);
		GPIO_write(color_S3_GPIO_Port,color_S3_Pin,1);
		//printf("\n--- photosensor set vert ---\r\n\n");
		break;
	case CLEAR:
		//clear
		GPIO_write(color_S2_GPIO_Port,color_S2_Pin,1);
		GPIO_write(color_S3_GPIO_Port,color_S3_Pin,0);
		//printf("\n--- photosensor set clear ---\r\n\n");
		break;
	case BLUE:
		//BLUE
		GPIO_write(color_S2_GPIO_Port,color_S2_Pin,0);
		GPIO_write(color_S3_GPIO_Port,color_S3_Pin,1);
		//printf("\n--- photosensor set blue ---\r\n\n");
		break;
	case RED:
		//RED
		GPIO_write(color_S2_GPIO_Port,color_S2_Pin,0);
		GPIO_write(color_S3_GPIO_Port,color_S3_Pin,0);
		//printf("\n--- photosensor set rouge ---\r\n\n");
		break;
	}
}

void colorSensorInit(h_color_sensor_t *h_color_sensor, color_sensor_color_t color, color_sensor_output_scale_t output_scale, color_sensor_state_t state){
	h_color_sensor->color=color;
	h_color_sensor->ouput_scale=output_scale;
	h_color_sensor->sensor_state=state;
	h_color_sensor->frequence=0;
	h_color_sensor->blue_color=0;
	h_color_sensor->green_color=0;
	h_color_sensor->red_color=0;

	h_color_sensor->green_transformation.green_coef_dir=1;
	h_color_sensor->green_transformation.green_min_freq=1;
	h_color_sensor->green_transformation.green_ord_origin=1;

	h_color_sensor->red_transformation.red_coef_dir=1;
	h_color_sensor->red_transformation.red_min_freq=1;
	h_color_sensor->red_transformation.red_ord_origin=1;

	h_color_sensor->calib_state=WAINTING_FOR_CALIB;
	h_color_sensor->calib_struct.calib_value_rouge_canette=1;
	h_color_sensor->calib_struct.calib_value_rouge_vide=1;
	h_color_sensor->calib_struct.calib_value_vert_canette=1;
	h_color_sensor->calib_struct.calib_value_vert_vide=1;
	colorSetOutputFreqScaling(h_color_sensor);
	colorSetPhotodiodeType(h_color_sensor,color);
	colorDisable(h_color_sensor);
}


// --- fonctions d'action ---

uint32_t colorGetRedValue(h_color_sensor_t * h_color_sensor){
	printf("sensor_value : %ld\r\n",h_color_sensor->frequence);
	return h_color_sensor->red_color;
}

uint32_t colorGetGreenValue(h_color_sensor_t * h_color_sensor){
	printf("sensor_value : %ld\r\n",h_color_sensor->frequence);
	return h_color_sensor->green_color;
}

void colorDisable(h_color_sensor_t * h_color_sensor){
	GPIO_write(color_enable_GPIO_Port,color_enable_Pin, 1);
	h_color_sensor->sensor_state=SENSOR_DISABLE;
	timer_handle(htim14,INPUT_CAPTURE_IT,STOP,TIM_CHANNEL_1);
}

void colorEnable(h_color_sensor_t * h_color_sensor){
	GPIO_write(color_enable_GPIO_Port,color_enable_Pin, 0);
	h_color_sensor->sensor_state=SENSOR_ENABLE;
	timer_handle(htim14,INPUT_CAPTURE_IT,START,TIM_CHANNEL_1);
}


// --- fonctions de calibration ---

uint32_t colorHandleCalibrationSensor(h_color_sensor_t * h_color_sensor){
	calibration_flag=1;
	printf("calibration flag : %d\r\n",calibration_flag);
	char entree[2];
	h_color_sensor->calib_state=CALIB_VERT_CANETTE;
	while(h_color_sensor->calib_state!=CALIB_DONE){
		//printf("entré dans while\r\n");
		switch(h_color_sensor->calib_state){
		case CALIB_VERT_CANETTE:
			colorSetPhotodiodeType(h_color_sensor,GREEN);

			// -- waiting for operator to put a green can in front of the sensor
			printf("press enter when a green can is captured\r\n");
			scanf( "%s",entree);
			printf("%s\r\n",entree);
			h_color_sensor->calib_struct.calib_value_vert_canette=0;
			colorEnable(h_color_sensor);
			while(h_color_sensor->calib_struct.calib_value_vert_canette==0){
				//Wait for calib completed
			}
			h_color_sensor->calib_state=CALIB_VERT_VIDE;
			break;

		case CALIB_VERT_VIDE:
			printf("calib_vert_vide\r\n");
			colorSetPhotodiodeType(h_color_sensor,GREEN);
			// -- waiting for operator to put a green can in front of the sensor
			printf("press enter when the green can is removed\r\n");
			scanf("%s",entree);
			h_color_sensor->calib_struct.calib_value_vert_vide=0;
			colorEnable(h_color_sensor);
			while(h_color_sensor->calib_struct.calib_value_vert_vide==0){
				//Wait for calib completed...
			}
			h_color_sensor->calib_state=CALIB_ROUGE_CANETTE;
			break;

		case CALIB_ROUGE_CANETTE:
			printf("calib_rouge_canette\r\n");
			colorSetPhotodiodeType(h_color_sensor,RED);

			// -- waiting for operator to put a green can in front of the sensor
			printf("press enter when a red can is captured\r\n");
			scanf("%s",entree);
			h_color_sensor->calib_struct.calib_value_rouge_canette=0;
			colorEnable(h_color_sensor);
			while(h_color_sensor->calib_struct.calib_value_rouge_canette==0){
				//Wait for calib completed...
			}
			h_color_sensor->calib_state=CALIB_ROUGE_VIDE;
			break;

		case CALIB_ROUGE_VIDE:
			printf("calib_rouge_vide\r\n");
			colorSetPhotodiodeType(h_color_sensor,RED);

			// -- waiting for operator to put a green can in front of the sensor
			printf("press enter when a green can is captured\r\n");
			scanf("%s",entree);
			h_color_sensor->calib_struct.calib_value_rouge_vide=0;
			colorEnable(h_color_sensor);
			while(h_color_sensor->calib_struct.calib_value_rouge_vide==0){
				//Wait for calib completed...
			}
			h_color_sensor->calib_state=CALIB_DONE;
			break;

		default:
			break;
		}
	}
	printf("calib_verte_canette : %u\r\n",h_color_sensor->calib_struct.calib_value_vert_canette);
	printf("calib_verte_vide : %u\r\n\n",h_color_sensor->calib_struct.calib_value_vert_vide);
	printf("calib_rouge_canette : %u\r\n",h_color_sensor->calib_struct.calib_value_rouge_canette);
	printf("calib_rouge_vide : %u\r\n\n",h_color_sensor->calib_struct.calib_value_rouge_vide);

	//computation of the transformation coefficient
	h_color_sensor->green_transformation.green_coef_dir=(uint16_t)((ECHELLE_VAL_HAUTE-ECHELLE_VAL_BASSE)/((h_color_sensor->calib_struct.calib_value_vert_vide)-(h_color_sensor->calib_struct.calib_value_vert_canette)));
	h_color_sensor->green_transformation.green_ord_origin=(uint16_t)(ECHELLE_VAL_HAUTE-((h_color_sensor->green_transformation.green_coef_dir)*(h_color_sensor->calib_struct.calib_value_vert_vide)));
	printf("coef dir vert : %u\r\nord origin vert : %u\r\nmin freq vert : %u\r\n\n",h_color_sensor->green_transformation.green_coef_dir,h_color_sensor->green_transformation.green_ord_origin,h_color_sensor->green_transformation.green_min_freq);

	h_color_sensor->red_transformation.red_coef_dir=(uint16_t)((ECHELLE_VAL_HAUTE-ECHELLE_VAL_BASSE)/((h_color_sensor->calib_struct.calib_value_rouge_vide)-(h_color_sensor->calib_struct.calib_value_rouge_canette)));
	h_color_sensor->red_transformation.red_ord_origin=(uint16_t)(ECHELLE_VAL_HAUTE-((h_color_sensor->red_transformation.red_coef_dir)*(h_color_sensor->calib_struct.calib_value_rouge_vide)));
	printf("coef dir rouge : %u\r\nord origin rouge : %u\r\nmin freq rouge : %u\r\n\n",h_color_sensor->red_transformation.red_coef_dir,h_color_sensor->red_transformation.red_ord_origin,h_color_sensor->red_transformation.red_min_freq);


	// flag to say hey calibration finished
	calibration_flag=0;
	return 0;
}


// --- fonction de gestion du retour capteur ---

void colorSensorHandleInputCapture_IT(h_color_sensor_t * h_color_sensor,TIM_TypeDef *TIM){
	//printf("entré dans tim2 IT\r\n");
	if(tour==0){
		temps1=TIM->CNT;
		tour=0;
	}
	if(tour==1){
		temps2=TIM->CNT;
		tour=1;
	}
	tour=1-tour;
	freq=abs(temps2-temps1);
	h_color_sensor->frequence=freq;

	/*// decommenter pour test calibration a la main
	calib_tab[u]=freq;
	u++;
	if(u==30){
		color_disable(h_color_sensor);
		for(int i=0;i<30;i++){
			printf("calib values[%d] : %ld\r\n",i,calib_tab[i]);
		}
		printf("fin affichage\r\n");
		u=0;
	}*/

	if((calibration_flag==1)&&(freq<FREQ_MAX_ACCEPTABLE)){
		colorHandleCalibrationValues(h_color_sensor,freq);
	}
	else if ((calibration_flag==0)&&(freq<FREQ_MAX_ACCEPTABLE)){
		// Analyse des valeur pour prise de decision sur la couleur
		colorHandleRawValues(h_color_sensor, freq);
	}
}

static int colorHandleCalibrationValues(h_color_sensor_t * h_color_sensor, uint32_t frequence){
	calib_tab[u]=freq;
	u++;
	if(u==CALIBRATION_NB_VALUES){
		colorDisable(h_color_sensor);
		uint32_t res=0;
		for(int i=0;i<30;i++){
			res=res+calib_tab[i];
			printf("calib values[%d] : %ld\r\n",i,calib_tab[i]);
		}
		switch(h_color_sensor->calib_state){
		case CALIB_VERT_CANETTE:
			h_color_sensor->calib_struct.calib_value_vert_canette=(uint16_t)(res/CALIBRATION_NB_VALUES);
			h_color_sensor->green_transformation.green_min_freq=h_color_sensor->calib_struct.calib_value_vert_canette;
			printf("Struct calib : champ vert_canette remplie\r\n");
			printf("Calib_value_vert_canette : %u\r\n\n",h_color_sensor->calib_struct.calib_value_vert_canette);
			break;
		case CALIB_VERT_VIDE:
			h_color_sensor->calib_struct.calib_value_vert_vide=(uint16_t)(res/CALIBRATION_NB_VALUES);
			printf("Struct calib : champ vert_vide remplie\r\n");
			printf("Calib_value_vert_vide : %u\r\n\n",h_color_sensor->calib_struct.calib_value_vert_vide);
			break;
		case CALIB_ROUGE_CANETTE:
			h_color_sensor->calib_struct.calib_value_rouge_canette=(uint16_t)(res/CALIBRATION_NB_VALUES);
			h_color_sensor->red_transformation.red_min_freq=h_color_sensor->calib_struct.calib_value_rouge_canette;
			printf("Struct calib : champ rouge_canette remplie\r\n");
			printf("Calib_value_rouge_canette : %u\r\n\n",h_color_sensor->calib_struct.calib_value_rouge_canette);
			break;
		case CALIB_ROUGE_VIDE:
			h_color_sensor->calib_struct.calib_value_rouge_vide=(uint16_t)(res/CALIBRATION_NB_VALUES);
			printf("Struct calib : champ rouge_vide remplie\r\n");
			printf("Calib_value_rouge_vide : %u\r\n",h_color_sensor->calib_struct.calib_value_rouge_vide);
			break;
		default:
			printf("bug dans la calibration\r\n\n");
			break;
		}
		u=0;
		return 1;
	}
	return 0;
}

static int colorHandleRawValues(h_color_sensor_t * h_color_sensor, uint32_t frequence){
	//printf("entre dans color_handle\r\n");
	if(raw_values_compteur<((NB_MEASURE_WANTED/2)-1)){
		// premieres valeurs du tableau donc on est dans le vert
		if(frequence<(h_color_sensor->green_transformation.green_min_freq)){
			frequence = (uint32_t)h_color_sensor->green_transformation.green_min_freq;
		}
		//--debug
		color_raw_values[raw_values_compteur]=frequence;
		//--fin debug
		color_scaled_values[raw_values_compteur]=((h_color_sensor->green_transformation.green_coef_dir)*frequence)+(h_color_sensor->green_transformation.green_ord_origin);
		raw_values_compteur++;
	}

	else if(raw_values_compteur==((NB_MEASURE_WANTED/2)-1)){
		//on est encore dans le vert mais on doit en plus changer la couleur de detection
		if(frequence<(h_color_sensor->green_transformation.green_min_freq)){
			frequence = (uint32_t)h_color_sensor->green_transformation.green_min_freq;
		}
		//--debug
		color_raw_values[raw_values_compteur]=frequence;
		//--fin debug
		color_scaled_values[raw_values_compteur]=((h_color_sensor->green_transformation.green_coef_dir)*frequence)+(h_color_sensor->green_transformation.green_ord_origin);
		raw_values_compteur++;

		// color change
		colorDisable(h_color_sensor);
		colorSetPhotodiodeType(h_color_sensor,RED);
		colorEnable(h_color_sensor);
	}

	else if((raw_values_compteur>((NB_MEASURE_WANTED/2)-1))&&(raw_values_compteur<NB_MEASURE_WANTED)){
		// valeures rouges
		if(frequence<(h_color_sensor->red_transformation.red_min_freq)){
			frequence = (uint32_t)h_color_sensor->red_transformation.red_min_freq;
		}
		//--debug
		color_raw_values[raw_values_compteur]=frequence;
		//--fin debug
		color_scaled_values[raw_values_compteur]=((h_color_sensor->red_transformation.red_coef_dir)*frequence)+(h_color_sensor->red_transformation.red_ord_origin);
		raw_values_compteur++;
	}

	else if(raw_values_compteur==NB_MEASURE_WANTED){
		// Pret a faire la moyenne du tableau pour plus de fiabilité
		colorDisable(h_color_sensor);
		uint32_t green_mean=0;
		uint32_t red_mean=0;

		// green mean with erase of waste values (the 2 first one each time)
		for(int i=DECHET_MEASURE; i<((NB_MEASURE_WANTED/2));i++){
			green_mean=green_mean+color_scaled_values[i];
		}
		green_color_value=floor(green_mean/((NB_MEASURE_WANTED/2)-DECHET_MEASURE));
		h_color_sensor->green_color=green_color_value;

		//red values mean
		for(int i=((NB_MEASURE_WANTED/2)+DECHET_MEASURE);i<NB_MEASURE_WANTED;i++){
			red_mean=red_mean+color_scaled_values[i];
		}
		red_color_value=floor(red_mean/((NB_MEASURE_WANTED/2)-DECHET_MEASURE));
		h_color_sensor->red_color=red_color_value;

		raw_values_compteur=0;

		//Affichage tableau
		for(int i=0;i<NB_MEASURE_WANTED;i++){
			printf("color_raw_values[%d] : %ld\t->\t color_scaled_value[%d] : %ld\r\n",i,color_raw_values[i],i,color_scaled_values[i]);
		}
		colorAnalyse(h_color_sensor,green_color_value,red_color_value);
	}

	return 0; // fonctionnement normal
}


// --- fonction d'analyse et de prise de décision ---

static void colorAnalyse(h_color_sensor_t * h_color_sensor, color_t green, color_t red){
	printf("green : %ld \r\n",green);
	printf("red : %ld \r\n",red);
	if(green<red){
		printf("c'est vert\r\n");
		/*if(freeRTOS){
			printf("Task resume all\r\n");
			xTaskResumeAll();
		}*/
	}
	else if(green==red){
		printf("Measure again\r\n");
		colorDoMeasureAgain(h_color_sensor);
	}
	else {
		printf("c'est rouge\r\n");
		/*if(freeRTOS){
			printf("task resume all\r\n");
			xTaskResumeAll();
		}*/
	}
}

static void colorDoMeasureAgain(h_color_sensor_t * h_color_sensor){
	colorEnable(h_color_sensor);
}


// --- FreeRTOS ---

extern TaskHandle_t h_colorMeasureTask;
extern SemaphoreHandle_t colorMeasureSemaphore;

static void colorMeasureTask(void * pvParameters){
	for(;;){
		xSemaphoreTake(colorMeasureSemaphore,portMAX_DELAY);
		//printf("-----contenu de colorMeasureTask------\r\n");
		// le semapore a été libéré
		h_color_sensor_t * h_color_sensor =(h_color_sensor_t *) pvParameters;

		//DEUX FONCTION A APPELLER POUR LIRE LA COULEUR
		colorSetPhotodiodeType(h_color_sensor, GREEN);
		colorEnable(h_color_sensor);
	}
}

int colorTaskCreation(h_color_sensor_t * h_color_sensor){

	//creation colorMeasureTask et colorMeasureSempahore
	if(pdTRUE==xTaskCreate(colorMeasureTask,"colorMeasureTask",COLOR_STACK_DEPTH,(void *)h_color_sensor,COLOR_MEASURE_TASK_PRIORiTY,&h_colorMeasureTask)){
		printf("colorMeasureTask created successfully\r\n");
	}
	else{
		printf("colorMeasureTask creation failed !\r\n");
		return 1;
	}
	colorMeasureSemaphore=xSemaphoreCreateBinary();
	if(colorMeasureSemaphore==NULL){
		printf("colorCalibSemaphore creation failed ! \r\n");
		return 1;
	}
	printf("colorMeasureSemaphore created successfully\r\n");
	return 0;
}




// Fonction d'abstraction de la lib HAL



/**
 * @fn void timer_handle(TIM_HandleTypeDef htim, tim_mode_t mode, tim_status_t status,uint32_t channel)
 * @brief Allow user to activate timer without using the buil-din funtions of STM. If the soft is to run on another hard device,
 * change only the build-in function of this timer_handle() instead of going through the code and change everithing.
 *
 * @param TIM_HandleTypeDef htim : structure of the timer you want to deal with
 * tim_mode_t mode : htim configuration accirding to what is in .ioc file (INPUT_CAPTURE_IT, BASE_IT, PWM)
 * tim_status_t status : same but START,STOP
 * uint32_t channel ; the output channel of htim
 * @return none
 */
void timer_handle(TIM_HandleTypeDef htim, tim_mode_t mode, tim_status_t status,uint32_t channel){
	if(status==START){
		switch(mode){
		case INPUT_CAPTURE_IT:
			HAL_TIM_IC_Start_IT(&htim, channel);
			break;
		case PWM:
			HAL_TIM_PWM_Start(&htim,channel);
			break;
		case BASE_IT:
			HAL_TIM_Base_Start_IT(&htim);
			break;
		default :
			break;
		}
	}
	else if (status==STOP){
		switch(mode){
		case INPUT_CAPTURE_IT:
			HAL_TIM_IC_Stop_IT(&htim, channel);
			break;
		case PWM:
			HAL_TIM_PWM_Stop(&htim,channel);
			break;
		case BASE_IT:
			HAL_TIM_Base_Stop_IT(&htim);
			break;
		default :
			break;
		}
	}
	else if (status==INIT){
		switch(mode){
		case INPUT_CAPTURE_IT:
			HAL_TIM_IC_Init(&htim);
			break;
		case PWM:
			HAL_TIM_PWM_Init(&htim);
			break;
		case BASE_IT:
			HAL_TIM_Base_Init(&htim);
			break;
		default :
			break;
		}
	}
}

/**
 * @fn int GPIO_write(GPIO_TypeDef * gpio_port,uint16_t gpio_pin,GPIO_PinState gpio_PinState )
 * @brief Allow user to set GPIO Pin without using the buil-din funtions of STM. If the soft is to run on another hard device,
 * change only the build-in function of this GPIO_write() instead of going through the code and change everithing.
 *
 * @param GPIO_TypeDef * gpio_port : the port of the GPIO you want to deal with
 * uint16_t gpio_pin : The pin of the GPIO you want to deal with
 * GPIO_PinState gpio_PinState : the state you want your GPIO to have (1 or 0)
 *
 * @return 0 if fail else 1
 */
void GPIO_write(GPIO_TypeDef * gpio_port,uint16_t gpio_pin,GPIO_PinState gpio_PinState ){
	HAL_GPIO_WritePin(gpio_port,gpio_pin,gpio_PinState);
}
