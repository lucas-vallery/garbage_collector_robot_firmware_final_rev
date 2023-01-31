/**
 * @file Color_sensor.h / Garbage Collector project
 * @brief Header file that contain the definition of objects used to operate of the color sensor
 * @author Baptiste Cottu
 * @date 19/12/2022
 */

#ifndef SRC_COLOR_SENSOR_H_
#define SRC_COLOR_SENSOR_H_

//liste des Pin en fin de fichier


// Macro necessaires a l'abstraction de la lib HAL
/*
#define color_S2_Pin GPIO_PIN_4
#define color_S2_GPIO_Port GPIOB
#define color_input_freq_Pin GPIO_PIN_15
#define color_input_freq_GPIO_Port GPIOA
#define button_Pin GPIO_PIN_11
#define button_GPIO_Port GPIOI
#define button_EXTI_IRQn EXTI15_10_IRQn
#define color_enable_Pin GPIO_PIN_7
#define color_enable_GPIO_Port GPIOC
#define color_S0_Pin GPIO_PIN_6
#define color_S0_GPIO_Port GPIOC
#define color_S3_Pin GPIO_PIN_7
#define color_S3_GPIO_Port GPIOG
#define color_S1_Pin GPIO_PIN_6
#define color_S1_GPIO_Port GPIOG
*/
#define NB_MEASURE_WANTED 20 			// Mettre un nombre pair, moyennage des valeur obtenues.
#define COLOR_STACK_DEPTH 500
#define COLOR_MEASURE_TASK_PRIORiTY 3


// --- fonction d'abstraction de la HAL ---
typedef enum tim_mode{
	INPUT_CAPTURE_IT = 1,
	BASE_IT = 2,
	PWM = 3
}tim_mode_t;
typedef enum tim_status{
	INIT=2,
	START = 1,
	STOP = 0
}tim_status_t;

void GPIO_write(GPIO_TypeDef * gpio_port,uint16_t gpio_pin,GPIO_PinState gpio_PinState );
void timer_handle(TIM_HandleTypeDef htim, tim_mode_t mode, tim_status_t status,uint32_t channel);


// ----- Declaration of the color sensor's usefull struct fields -----
typedef enum color_sensor_color_enum{
	RED =1,
	BLUE=2,
	CLEAR=3,
	GREEN=4
}color_sensor_color_t;
typedef enum color_sensor_output_scale_enum{
	CENT_POUR_CENT=4,
	VINGT_POUR_CENT=3,
	DEUX_POUR_CENT=2,
	POWER_DOWN=1
}color_sensor_output_scale_t;
typedef enum color_sensor_state_enum{
	SENSOR_ENABLE=1,
	SENSOR_DISABLE=0
}color_sensor_state_t;
typedef enum calibration_state_enum{
	WAINTING_FOR_CALIB=0,
	CALIB_VERT_CANETTE=1,
	CALIB_VERT_VIDE=2,
	CALIB_ROUGE_CANETTE=3,
	CALIB_ROUGE_VIDE=4,
	CALIB_DONE=5
}calibration_state_t;

/**
 * \struct h_green_transormation_t
 * \brief Contain the coeficient to transfomr the green values
 */
typedef struct h_green_transormation_t{
	uint16_t green_coef_dir;
	uint16_t green_ord_origin;
	uint16_t green_min_freq;
}h_green_transformation_t;

/**
 * \struct h_red_transormation_t
 * \brief Contain the coeficient to transfomr the red values
 */
typedef struct h_red_transformation_t{
	uint16_t red_coef_dir;
	uint16_t red_ord_origin;
	uint16_t red_min_freq;
}h_red_transformation_t;

/**
 * \struct h_calibration_structure_t
 * \brief Calibration object. Contain raw values of the sensor calibration
 */
typedef struct h_calibration_buffer_struct_t{
	uint16_t calib_value_vert_canette;
	uint16_t calib_value_vert_vide;
	uint16_t calib_value_rouge_canette;
	uint16_t calib_value_rouge_vide;
}h_calib_buffer_structure_t;

typedef uint32_t color_t;


// ----- Declaration of the color sensor structure -----
/**
 * \struct h_color_sensor_t
 * \brief Color sensor object
 *
 * This object contains all the features of this color sensor. It allow user to manage more than 1 color sensor at a time and deal with them easily
 */
typedef struct h_color_sensor_t{
	color_sensor_color_t color;
	color_sensor_output_scale_t ouput_scale;
	color_sensor_state_t sensor_state;

	uint32_t frequence;

	color_t green_color;
	color_t	red_color;
	color_t blue_color;

	calibration_state_t calib_state;

	h_green_transformation_t green_transformation;
	h_red_transformation_t red_transformation;
	h_calib_buffer_structure_t calib_struct;
}h_color_sensor_t;


// ----- Color sensor functions -----
void colorSensorInit(h_color_sensor_t * h_color_sensor, color_sensor_color_t color, color_sensor_output_scale_t output_scale, color_sensor_state_t state);

/**
 * @fn int colorEnable(h_color_sensor_t * h_color_sensor)
 * @brief Enable the sensor by setting to 0 the ENABLE pin
 *
 * @param h_color_sensor : color_sensor structure
 * @return 0 if failed else 1
 */
void colorEnable(h_color_sensor_t * h_color_sensor);

/**
 * @fn int colorDisable(h_color_sensor_t * h_color_sensor)
 * @brief disable the sensor by setting to 1 the ENABLE pin
 *
 * @param h_color_sensor : color_sensor structure
 * @return 0 if failed else 1
 */
void colorDisable(h_color_sensor_t * h_color_sensor);

/**
 * @fn uint32_t colorGetGreenValue(h_color_sensor_t * h_color_sensor)
 * @brief Fast acces to the green value captured by the sensor h_color_sensor
 *
 * @param h_color_sensor : color_sensor structure
 * @return 0 if failed else 1
 */
uint32_t colorGetGreenValue(h_color_sensor_t * h_color_sensor);

/**
 * @fn uint32_t colorGetRedValue(h_color_sensor_t * h_color_sensor)
 * @brief Fast acces to the red value captured by the sensor h_color_sensor
 *
 * @param h_color_sensor : color_sensor structure
 * @return 0 if failed else 1
 */
uint32_t colorGetRedValue(h_color_sensor_t * h_color_sensor);

/**
 * @fn void colorSensorHandleInputCapture_IT(h_color_sensor_t * h_color_sensor, TIM_TypeDef* TIM,h_calib_buffer_structure_t * h_calib_buffer_struct);
 * @brief handle the input capture interruption. Manage if the sensor is being operated of calibrated
 *
 * @param h_color_sensor : color_sensor structure
 * @param TIM: Timer used for the input capture
 * @param h_calib_buffer_struct : buffer calibration structure used in case of the sensor being calibrated
 * @return 0 if failed else 1
 */
void colorSensorHandleInputCapture_IT(h_color_sensor_t * h_color_sensor, TIM_TypeDef* TIM);

/**
 * @fn uint32_t colorHandleCalibrationSensor(h_color_sensor_t * h_color_sensor);
 * @brief handle the input capture interruption. Manage if the sensor is being operated of calibrated
 *
 * @param h_color_sensor : color_sensor structure
 * @return 0 if failed else 1
 */
uint32_t colorHandleCalibrationSensor(h_color_sensor_t * h_color_sensor);

/**
 * @fn int colorSetPhotodiodeType(h_color_sensor_t * h_color_sensor)
 * @brief Set the photoreceptor of the sensor. Each sensor can detect color only one by one.
 * User have to set different types of filters to detect the roght color
 *
 * @param h_color_sensor : color_sensor structure
 * @return 0 if failed else 1
 */
void colorSetPhotodiodeType(h_color_sensor_t * h_color_sensor,color_sensor_color_t color);

/**
 * @fn void colorStartSensor(h_color_sensor_t h_color_sensor)
 * @brief Start task dealing with the colorSensor
 *
 * @param (void *) color_sensor
 * @return 1 if failed else 0
 */
int colorTaskCreation(h_color_sensor_t * h_color_sensor);


/*  liste des Pins
 *		color_S0	GPIO_output
 *	    color_S1	GPIO_output
 *		color_S2	GPIO_output
 *		color_S3	GPIO_output
 *		color_enable GPIO_Output
 *		3,3V ou 5V (*2)
 *		GND
 *		color_freq	INPUT_CAPTURE
 *
 *		1 timer en input capture
 *	    Voir si un timer est dispo pour déclencher tout les demi secondes le changelnt de filtre ou alors 4 cycle de IC et on passe au filtre suivant : je gererai ça une fois en programation
 */

#endif /* SRC_COLOR_SENSOR_H_ */
