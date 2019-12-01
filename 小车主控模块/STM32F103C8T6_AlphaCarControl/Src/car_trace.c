#include "main.h"
#include "stm32f1xx_hal.h"
#include "car_motor.h"
#include "car_trace.h"

//五个探头所对应的IO口说明
// OUT1 | OUT2 | OUT3 | OUT4 | OUT5
// L2		-	L1	 - M    - R1   - R2
#define OUT1   GPIO_PIN_3  
#define OUT2   GPIO_PIN_4
#define OUT3   GPIO_PIN_5
#define OUT4   GPIO_PIN_6
#define OUT5   GPIO_PIN_7

#define bitSet(a,n) 	((a) = (a) | (0x1<<(n))) 		//bitSet(irs[j], i); // 设置irs[j]的第ibit 为1
#define bitClear(a,n) ((a) = (a) & ~(0x1<<(n)))		//bitClear(irs[j], i); //清除irs[j]的第ibit为0
//constrain()函数的功能：若是	val > val_max,取 val_max，
//                       若是	val < val_min,取 val_min,
//             					 若是	val_min<=	val <= val_max,取*pvalue
#define constrain(val,val_min,val_max) 	((val)<(val_min) ? (val_min) : ((val) > (val_max) ? (val_max) : (val)))
#define abs(val) ((val) > 0 ? (val) : -(val))

const float motorSpeed = 999; 
const int IR_PIN[] = {OUT5, OUT4, OUT3, OUT2, OUT1}; //  寻迹传感器接口,从右往左
pid_t pid;
float pidValue = 0;
Bool turnFlag = false;

//小车寻迹初始化
void car_trace_init()
{
	//PID控制参数初始化
  pid.sampleTime = SAMPLE_TIME;
  pid.Kp = KP_VALUE;
  pid.Ki = KI_VALUE;
  pid.Kd = KD_VALUE;
  pid.error = 0;
  pid.previous_error = 0;
  
	set_left_PWM_duty((uint16_t) motorSpeed);
	set_right_PWM_duty((uint16_t) motorSpeed);
}

//get ir data and middle filter
uint8_t getIrData(void)  
{
  int i, j;
  uint8_t level;
  uint8_t temp;
  uint8_t irs[9] = {0};//采集9次传感器的传感器的数据
             //减少取平均值，滤波。

  for(j = 0; j < 9; j ++){
		for (i = 0; i < 5; i++){
			//level = digitalRead(IR_PIN[i]);
			level = HAL_GPIO_ReadPin(GPIOB, IR_PIN[i]);
			if(level){
				bitSet(irs[j], i); // 设置irs[j]的第ibit 为1
			}else{
				bitClear(irs[j], i); //清除irs[j]的第ibit为0
			}
		}
  }
  //冒泡法，对irs中的数据，进行从小到大排序，取最中间的值。
  for (i = 0; i < 9 - 1; i ++) {
    for (j = 0; j < 9 - i - 1; j ++) {
      if (irs[j] > irs[j + 1]) {
        temp = irs[j];
        irs[j] = irs[j + 1];
        irs[j + 1] = temp;
      }
    }
  }
  
  return irs[4];
}

int calcErrorByIrsValue(uint8_t irs)
{
  int curError = pid.error; //获得上一次的pid误差值
	
	//【小车左偏】右边压线为负; 【小车右偏】左边压线为正 
  //偏差为负值，小车需右转修正；偏差为正值，小车需左转修正
  //压线输出高电平1，没有压线输出低电平0
  switch(irs){ 
		//最右边压线
    case 0x01: curError = -8;  //0B00001
									break; 
		//右四个或者右3个沿线
    case 0x07:               //0B00111  
    case 0x0f: curError = -7; 	 //0B01111
									break;
		//右两个沿线
    case 0x03: curError = -6;  //0B00011
									break; 
		//右倒数第二个沿线
    case 0x02: curError = -4;  //0B00010
									break; 
    case 0x06: curError = -2; //0B00110
									break; 
		 //全部压线和中间压线，对应"十字"弯道，直接郭即可。
    case 0x1f:             //0B11111   
    case 0x04: curError = 0; //0B00100
									break;
    case 0x0c: curError = 2; //0B01100
									break;
    case 0x08: curError = 4; //0B01000
									break;
    case 0x18: curError = 6; //0B11000
									break;
    case 0x1c:							//0B11100
    case 0x1e: curError = 7; //0B11110
									break;
    case 0x10: curError = 8; //0B10000
									break;
		//都没有沿线，按照上次的PID误差值来调控
    case 0x00: curError = curError > 0 ? 9 : - 9; 
									break;
  }

  return curError;
}


//数据排序
void _sortData(int *p, int n)
{
  int temp;
  int i, j;
  
  for(i = 0; i < n - 1; i ++){
    for(j = 0; j < n - i - 1; j++){
      if(p[j] > p[j + 1]){
        temp = p[j];
        p[j] = p[j + 1];
        p[j + 1] = temp;
      }
    }
  }

  return;
}


//计算当前的误差值，得到当前的偏移值
void calcCurrentError(void)
{
  int i;
  uint8_t irs;
  float sum = 0;
  int errorData[10];

  for (i = 0; i < 10; i ++) {
    irs =  getIrData();  //得到这次传感器采集的数据（5路数据）
    errorData[i] =  calcErrorByIrsValue(irs);
  }

  _sortData(errorData, 10);

	//去掉最小和最大值，求平均值
  for (i = 1; i < 10 - 1; i++){
    sum += errorData[i];
  }
  pid.error = sum / 8;

}

//小车右转
void turnRight(void)
{
   //car_right_in_position();
}

//小车左转
void turnLeft(void)
{ 
  //car_left_in_position();
}

//小车前进
void goForward(void)
{
   //car_foreward();
}

void motorControl(float pidValue, Bool turnFlag)
{
  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;
  
  leftMotorSpeed  = constrain((motorSpeed - pidValue), -999, 999);
  rightMotorSpeed = constrain((motorSpeed + pidValue), -999, 999);

  if(turnFlag == true){			//极限情况处理
    if(abs(leftMotorSpeed) > abs(rightMotorSpeed)){
      leftMotorSpeed  = abs(leftMotorSpeed);
      rightMotorSpeed = leftMotorSpeed;
    }else{
      rightMotorSpeed =  abs(rightMotorSpeed);
      leftMotorSpeed  = rightMotorSpeed;
    }
  }else{									//一般情况处理
    leftMotorSpeed  = leftMotorSpeed  > 0 ? leftMotorSpeed  : -leftMotorSpeed;
    rightMotorSpeed = rightMotorSpeed > 0 ? rightMotorSpeed : -rightMotorSpeed;
  }

	
	set_left_PWM_duty((uint16_t) leftMotorSpeed);
	set_right_PWM_duty((uint16_t) rightMotorSpeed);
	
  return;
}

Bool calculatePid(float *pValue)
{
  float P = 0;
  static float I = 0 ;
  float D = 0 ;
  static unsigned long lastTime = 0;
  //获得机器从启动到现在所运行的时间
  //unsigned long now = millis(); 
	static unsigned long now = 0;
  int timeChange = 0;

	now++;
	timeChange = now - lastTime;//得到这次的时间
	
  //若是我们的这次变化的时间，小于pid的
  //采样时间，就需要等待一下。
  if (timeChange < pid.sampleTime) {
    return false;
  }

  P = pid.error; //本次小车传感器的偏移大小
  I = I + pid.error;//历史的偏移
  D = pid.error - pid.previous_error; //最近两次的差值

  *pValue = (pid.Kp * P) + (pid.Ki * I) + (pid.Kd * D) + 1;
  
  *pValue = constrain(*pValue, -motorSpeed,motorSpeed);

  pid.previous_error = pid.error; //记录本次的pid.error，作为下一次的历史差值。
  lastTime = now; // 记录本次的运行时间，作为下一次时间的历史时间参考。

  return true;
}


//判断小车的转向
void calcDirection(void) 
{
 
  //pid.error > 0 说明小车左边压线多，右偏要往左修正
  if (pid.error >= 8 && pid.error <= 9) {
		turnLeft();
    turnFlag = true;
  } else if (pid.error >= -9 && pid.error <= -8) {
    turnRight();
    turnFlag = true;
  } else {
    goForward();
    turnFlag = false;
  }

  return;
}

//小车寻迹控制
void car_trace_control(void)
{
  Bool ok;
  float  pidValue;

  calcCurrentError();
  ok = calculatePid(&pidValue); //计算当前pid的误差值对应的控制量
  if(ok){
    calcDirection();
    motorControl(pidValue, turnFlag);
  }

 
}












