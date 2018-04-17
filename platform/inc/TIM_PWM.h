#ifndef _TIM_PWM_H
#define _TIM_PWM_H
#include "stdint.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#define M1_FORWARD 0
#define M1_REVERSE 1
#define M2_FORWARD 1
#define M2_REVERSE 0
#define M3_FORWARD 1
#define M3_REVERSE 0
#define M4_FORWARD 1
#define M4_REVERSE 0

#define LEFT 0
#define RIGHT 1
#define MOTOR_LEFT_OR_RIGHT RIGHT

typedef struct{
		float duty;
		uint16_t dutyRange;
		uint16_t dutyValue;
		uint8_t dir;
		uint8_t inverse;   /*该变量用于左右轮的坐标变换，比如左轮前进方向为GPIO口设置为1，那么右轮的前进方向为GPIO口设置为0*/
		uint8_t workStatus;
		GPIO_TypeDef* CW_GPIOx;
		uint16_t CW_GPIO_Pin;
		TIM_TypeDef* TIMx;
		volatile uint16_t* CCRx;
}MotorParam;

extern MotorParam motor1, motor2, motor3, motor4;

void Timer3_PWMConfig(void);
void MotorControl_Init(void);
void EXTI_Dir_Config(void);
void setMotorDir(MotorParam* motor, uint8_t dir);
void setMotorSpeed(MotorParam* motor, uint16_t value);
void setMotorDirSpeed(MotorParam* motor, int16_t speed_dir);

#endif
