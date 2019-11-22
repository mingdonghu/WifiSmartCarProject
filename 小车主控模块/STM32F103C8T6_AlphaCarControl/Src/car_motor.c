/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,Alpha Car Team
 * All rights reserved.
 *
 * @file          car_motor.c
 * @brief         小车电机驱动程序
 * @company       阿尔法车队
 * @author        胡明栋
 * @version       v1.5
 * @Software    MDK5,STM32CubeMX,STM32Cube HAl 库
 * @Target core   stm32f103c8t6	
 * @date          2019-11-21
 ********************************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "car_motor.h"

extern TIM_HandleTypeDef htim1;

//@函数功能:	电机PWM调速初始化
void PWM_init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
}

//@函数功能：							左电机速度设置
//@left_motor_duty_val:		左电机占空比值 (0~999)
void set_left_PWM_duty(uint16_t left_motor_duty_val)
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,left_motor_duty_val); //左电机 ENA PA11 PWM占空比(0~999)
}

//@函数功能：							右电机速度设置
//@right_motor_duty_val:	右电机占空比值 (0~999)
void set_right_PWM_duty(uint16_t right_motor_duty_val)
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,right_motor_duty_val); //右电机 ENB PA10 PWM占空比(0~999)
}

//@函数功能:电机正转(与车向一致)
//@motor_type:		ML / MR
void motor_forward_rotation(Motor_t motor_type)
{
	if(motor_type == ML){
	//左电机 正转
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //IN3 PA4 1
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //N4 PA5 0
	
	}else if(motor_type == MR){
		//右电机 正转
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  //IN1 PB0 1
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); //IN2 PB1 0 
	}
	
	
}

//@函数功能:电机反转(与车向一致)
//@motor_type:		ML / MR
void motor_reversal_rotation(Motor_t motor_type)
{
	if(motor_type == ML){
			//左电机 反转
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // IN3 PA4  0
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // IN4 PA5  1
		
	}else if(motor_type == MR){
			//右电机 反转
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //IN1 PB0  0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   //IN2 PB1  1
	}
}

//#函数功能:控制小车停止
void car_stop(void)
{
	//左右电机停
	//__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,999); //左电机 ENA PA11 PWM占空比(0~999)  一直高电平
	//__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,999); //右电机 ENB PA10 PWM占空比(0~999)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // IN3 PA4  0
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   // IN4 PA5  0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //IN1 PB0  0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);   //IN2 PB1  0
	
}

//@函数功能：控制小车前进
void car_foreward(void)
{
	motor_forward_rotation(ML);
	motor_forward_rotation(MR);
}

//@函数功能：控制小车后退
void car_backward(void)
{
	motor_reversal_rotation(ML);
	motor_reversal_rotation(MR);
}

//@函数功能：控制小车原地左转
void car_left_in_position(void)
{
  motor_reversal_rotation(ML);	//左反
	motor_forward_rotation(MR);		//右正
}

//@函数功能：控制小车原地右转
void car_right_in_position(void)
{
 	motor_forward_rotation(ML);		//左正
	motor_reversal_rotation(MR);	//右反
}
