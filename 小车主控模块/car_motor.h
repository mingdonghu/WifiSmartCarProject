#ifndef __CAR_MOTOR_H
#define __CAR_MOTOR_H

#include "stdint.h"

/*******************************************************************************/
void PWM_init();
void set_left_PWM_duty(uint16_t left_motor_duty_val);
void set_right_PWM_duty(uint16_t right_motor_duty_val);
void car_foreward();
void car_backward();
void car_left_in_position();
void car_right_in_position();
/******************************************************************************/

#endif
