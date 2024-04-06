#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "motor.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "gfly.h"

void  motorInit(void);
void  setMotor(u8 Value1,u8 Value2,u8 Value3,u8 Value4);
void setMotor_H_def(int Value1,int Value2,int Value3,int Value4);



#endif

