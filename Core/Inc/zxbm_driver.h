/*
 * zxbm_driver.h
 *
 *  Created on: Dec 1, 2022
 *      Author: lucas
 */

#ifndef INC_ZXBM_DRIVER_H_
#define INC_ZXBM_DRIVER_H_

#include <stdint.h>

typedef void (* h_timer_driver_start_stop_pwm_t)(void);
typedef void (* h_timer_driver_set_duty_cycle_t)(int dutyCycle);

typedef enum {
	noReversed = 0,
	reversed = 1,
}h_isRevesrsed_t;

typedef struct {
	h_timer_driver_start_stop_pwm_t startPWM;
	h_timer_driver_start_stop_pwm_t startPWMN;

	h_timer_driver_start_stop_pwm_t	stopPWM;
	h_timer_driver_start_stop_pwm_t stopPWMN;

	h_timer_driver_set_duty_cycle_t setDutyCycle;
}h_timer_driver_t;

typedef struct {
	h_timer_driver_t timer;
	h_isRevesrsed_t isReversed;
}h_zxbm_t;

typedef enum {
	zxbm_forward    = 0,
	zxbm_backward   = 1,
	zxbm_hold		= 2,
	zxbm_stop		= 3
}h_zxbm_direction_t;

void zxbm_run(h_zxbm_t* driver, h_zxbm_direction_t dir);
void zxbm_setDutyCycle(h_zxbm_t* driver, int dutyCycle);

#endif /* INC_ZXBM_DRIVER_H_ */
