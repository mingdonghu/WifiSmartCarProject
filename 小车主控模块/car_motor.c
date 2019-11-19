/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "car_motor.h"

extern TIM_HandleTypeDef htim2;

//@函数功能:	电机PWM调速初始化
//@返回值: 		无
void PWM_init()
{
	MX_TIM2_Init();  //在main.h文件中被声明
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
}

//@函数功能：							左电机速度设置
//@left_motor_duty_val:		左电机占空比值 (0~999)
//@返回值: 无
void set_left_PWM_duty(uint16_t left_motor_duty_val)
{
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,left_motor_duty_val); //左电机 ENA PA0 PWM占空比(0~999)
}

//@函数功能：							右电机速度设置
//@right_motor_duty_val:	右电机占空比值 (0~999)
//@返回值: 无
void set_right_PWM_duty(uint16_t right_motor_duty_val)
{
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,right_motor_duty_val); //右电机 ENB PA3 PWM占空比(0~999)
}


//@函数功能：控制小车前进
//@输入参数：  无
//@返回值： 无
void car_foreward()
{
	//小车前进						[qian]
		//左电机 反转
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); //IN1 PA2  0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   //IN2 PB9  1
		//右电机 反转
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // IN3 PA1  0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   // IN4 PC9  1
		
	//HAL_Delay(10000);
}

//@函数功能：控制小车后退
//@输入参数：  无
//@返回值： 无
void car_backward()
{
		//小车后退           [hou]
		//左电机 正转
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);  //IN1 PA2 1
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); //IN2 PB9 0 
		//右电机 正转
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  //IN3 PA1 1
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); //N4 PC9 0
	 
	 //HAL_Delay(1000); //延时1000ms
}

//@函数功能：控制小车原地左转
//@输入参数：  无
//@返回值： 无
void car_left_in_position()
{
  //小车左转							[zuo]
		//左电机 正转
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);  //IN1 PA2  1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); //IN2 PB9  0
		 //右电机 反转
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  //IN3 PA1  0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);    //IN4 PC9  1
	
	//HAL_Delay(1000);
}

//@函数功能：控制小车原地右转
//@输入参数：  无
//@函数返回值： 无
void car_right_in_position()
{
 		//小车右转            [you]
	 //左电机 反转
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);  //IN1 PA2  0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);    // IN2 PB9  1
	 //右电机 正转
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  //IN3 PA1  1
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // IN4 PC9  0
		
	//HAL_Delay(1000);
}
