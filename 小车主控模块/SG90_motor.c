/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "SG90_motor.h"

extern TIM_HandleTypeDef htim3;

//@函数功能：摄像头转动控制初始化
void SG90_motor_init()
{
	MX_TIM3_Init();	 //舵机转角PWM
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);	//PA6 -P22 UD_SG90Motor
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);	//PA7	-p23 LR_SG90Motor
}

//@函数功能：摄像头上下转动,PWM占空比控制设定
//@val：	45~135
/*
	45  正看 --UD_FlatLook_val
	135	俯看 --UD_OverLook_val
*/
void SG90_motor_set_UD_val(uint16_t val)
{
	
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,val); //PA6 -P22 UD_SG90Motor		45~225(0度upMAX~180度downMAX)
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
	
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,val); //PA7 -p23 LR_SG90Motor		45~225(0度rightMAX~180度leftMAX)
																									//LR_MID_VAL = 123		中间
																									//LR_RIGHT_VAL = 48		最右
																									//LR_LEFT_VAL = 198		最左
}

//@函数功能：摄像头动作测试
void SG90_TEST()
{
	int i;
	
	for(i = 123; i >= 48; i-- ){
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,i);
				HAL_Delay(50);
		}
		
		for(i = 48; i <= 123; i++ ){
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,i);
				HAL_Delay(50);
		}
		
		for(i = 123; i <= 198; i++ ){
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,i);
				HAL_Delay(50);
		}
		
		for(i = 198; i >= 123; i-- ){
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,i);
				HAL_Delay(50);
		}
		
		for(i = 45; i <= 135; i++){
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,i);
				HAL_Delay(50);
		}
		
		for(i = 135; i >= 45; i--){
				__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,i);
				HAL_Delay(50);
		}
}