#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
#define ZHONGZHI 3100
#define FILTERING_TIMES  4

#define POSITION_MIDDLE 7925
#define ANGLE_ORIGIN 1020
#define ANGLE_MIDDLE 3100

/* R4-5: ceiling on how much PWM the angle-loop integral term may contribute.
   Expressed in PWM, not in integral units, so it does not move when a
   student changes Ki. 2000 of 6900 full scale: enough to remove a real
   offset, not enough for a wound-up integrator to slam the cart. */
#define BALANCE_I_PWM_LIMIT 2000.0f
extern	int Balance_Pwm,Velocity_Pwm;

extern float Bias;                       //倾角偏差
extern float Last_Bias,D_Bias;    //PID相关变量
extern int balance;                     //PWM返回值 

extern float Position_PWM,Last_Position,Position_Bias,Position_Differential;
extern float Position_Least;

int TIM1_UP_IRQHandler(void);
int Balance(float angle);
int Position(int Encoder);
void Set_Pwm(int moto);
void Key(void);
void Xianfu_Pwm(void);
u8 Turn_Off(int voltage);
int myabs(int a);
void Find_Zero(void);
void Auto_run(void);
int Incremental_PI (int Encoder,int Target);
void Get_D_Angle_Balance(void);
int Mean_Filter(int sensor);
int Position_PID (int Encoder,int Target);
int Pre_Position(int Encoder);
#endif
