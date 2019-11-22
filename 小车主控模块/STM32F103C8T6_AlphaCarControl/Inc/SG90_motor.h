/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,Alpha Car Team
 * All rights reserved.
 *
 * @file          SG90_motor.h
 * @brief         摄像头云台驱动程序
 * @company       阿尔法车队
 * @author        胡明栋
 * @version       v1.5
 * @Software    MDK5,STM32CubeMX,STM32Cube HAl 库
 * @Target core   stm32f103c8t6	
 * @date          2019-11-21
 ********************************************************************************************************************/

#ifndef __SG90_MOTOR_H
#define __SG90_MOTOR_H

#include "stdint.h"

#define UD_FLAT_LOOK_VAL 45   //正看占空比值
#define UD_OVERlOOK_VAL  135	//俯看占空比值

#define LR_MID_VAL 123	//中间
#define LR_RIGHT_VAL 48	//最右
#define LR_LEFT_VAL 198	//最左

/*********************************/
void SG90_motor_init(void);
void SG90_motor_set_UD_val(uint16_t val);
void SG90_motor_set_LR_val(uint16_t val);
void camera_TEST(void);
void camera_left_move(void);
void camera_right_move(void);
void camera_up_move(void);
void camera_down_move(void);

#endif
