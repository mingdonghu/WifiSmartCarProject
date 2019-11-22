/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,Alpha Car Team
 * All rights reserved.
 *
 * @file          car_motor.h
 * @brief         小车电机驱动程序
 * @company       阿尔法车队
 * @author        胡明栋
 * @version       v1.5
 * @Software    MDK5,STM32CubeMX,STM32Cube HAl 库
 * @Target core   stm32f103c8t6	
 * @date          2019-11-21
 ********************************************************************************************************************/

#ifndef __CAR_MOTOR_H
#define __CAR_MOTOR_H

#include "stdint.h"


typedef enum{
	ML = 0,	/*左电机*/
	MR = 1,	/*右电机*/
}Motor_t;

/*******************************************************************************/
void PWM_init();
void set_left_PWM_duty(uint16_t left_motor_duty_val);
void set_right_PWM_duty(uint16_t right_motor_duty_val);
void car_stop();
void car_foreward();
void car_backward();
void car_left_in_position();
void car_right_in_position();
void motor_forward_rotation(Motor_t motor_type);
void motor_reversal_rotation(Motor_t motor_type);
/******************************************************************************/

#endif
