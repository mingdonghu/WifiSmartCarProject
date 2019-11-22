/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,Alpha Car Team
 * All rights reserved.
 *
 * @file          SG90_motor.c
 * @brief         摄像头云台驱动程序
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
#include "SG90_motor.h"

extern TIM_HandleTypeDef htim2;

uint16_t rlVal = LR_MID_VAL;					//摄像头左右转动控制值
uint16_t udVal = UD_FLAT_LOOK_VAL;		//摄像头上下转动控制值


//@函数功能：摄像头转动控制初始化
void SG90_motor_init()
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);	//PA0  UD_SG90Motor
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);	//PA1	 LR_SG90Motor
}

//@函数功能：摄像头上下转动,PWM占空比控制设定
//@val：	45~135
/*
	45  正看 --UD_FlatLook_val
	135	俯看 --UD_OverLook_val
*/
void SG90_motor_set_UD_val(uint16_t val)
{
	
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,val); //PA0  UD_SG90Motor		45~225(0度upMAX~180度downMAX)
																									//UD_FlatLook_val = 45  正看
																									//UD_OverLook_val = 135	俯看
}

//@函数功能：摄像头左右转动,PWM占空比控制设定
//@val：48~123~198
/*
			48		最右LR_RIGHT_VAL
			123		中间LR_MID_VAL
			198		最左LR_LEFT_VAL
*/
void SG90_motor_set_LR_val(uint16_t val)
{
	
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,val); //PA1  LR_SG90Motor		45~225(0度rightMAX~180度leftMAX)
																									//LR_MID_VAL = 123		中间
																									//LR_RIGHT_VAL = 48		最右
																									//LR_LEFT_VAL = 198		最左
}

//@函数功能:摄像头左转
void camera_left_move()
{
		rlVal = rlVal + 3;
	if(rlVal >= LR_LEFT_VAL){
		rlVal = LR_LEFT_VAL;
	}
	SG90_motor_set_LR_val(rlVal);
	//HAL_Delay(1); //delay 1ms
}

//@函数功能:摄像头右转
void camera_right_move()
{
		rlVal = rlVal - 3;
	if(rlVal <= LR_RIGHT_VAL){
		rlVal = LR_RIGHT_VAL;
	}
	SG90_motor_set_LR_val(rlVal);
	//HAL_Delay(1); //delay 1ms
}

//@函数功能:摄像头上转
void camera_up_move()
{
	udVal = udVal - 3;
	if(udVal <= UD_FLAT_LOOK_VAL){
			udVal = UD_FLAT_LOOK_VAL;
	}
	SG90_motor_set_UD_val(udVal);
	//HAL_Delay(1); //delay 1ms
}

//@函数功能:摄像头下转
void camera_down_move()
{
	udVal = udVal + 3;
	if(udVal >= UD_OVERlOOK_VAL){
		udVal = UD_OVERlOOK_VAL;
	}
	SG90_motor_set_UD_val(udVal);
	//HAL_Delay(1); //delay 1ms
}


//@函数功能：摄像头动作测试
void camera_TEST()
{
	int i;
	
	for(i = 123; i >= 48; i-- ){
				__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,i);
				HAL_Delay(50);
		}
		
		for(i = 48; i <= 123; i++ ){
				__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,i);
				HAL_Delay(50);
		}
		
		for(i = 123; i <= 198; i++ ){
				__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,i);
				HAL_Delay(50);
		}
		
		for(i = 198; i >= 123; i-- ){
				__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,i);
				HAL_Delay(50);
		}
		
		for(i = 45; i <= 135; i++){
				__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,i);
				HAL_Delay(50);
		}
		
		for(i = 135; i >= 45; i--){
				__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,i);
				HAL_Delay(50);
		}
}