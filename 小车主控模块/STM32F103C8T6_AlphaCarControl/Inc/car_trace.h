#ifndef _CAR_TRACE_H
#define _CAR_TRACE_H

#include "stdint.h"

typedef enum{
	false = 0,
	true = 1,
}Bool;

typedef struct {
  float Kp;  //比例系数
  float Ki;   //积分时间常数
  float Kd;  //微分时间常数
  float error;
  int sampleTime; //采样时间
  float previous_error; 
} pid_t;


#define SAMPLE_TIME 10
#define KP_VALUE  30
#define KI_VALUE  0.03
#define KD_VALUE  13


void car_trace_init(void);//小车寻迹初始化
uint8_t getIrData(void);  //get ir data and middle filter
int calcErrorByIrsValue(uint8_t irs);
void _sortData(int *p, int n);  //数据排序
void calcCurrentError(void);  //计算当前的误差值，得到当前的偏移值
void turnRight(void); //小车右转
void turnLeft(void); //小车左转
void goForward(void);  //小车前进
void motorControl(float pidValue, Bool turnFlag);
Bool calculatePid(float *pValue);
void calcDirection(void); //判断小车的转向
void car_trace_control(void);//小车寻迹控制


#endif
