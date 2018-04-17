#include "stm32f10x.h"
#include "TIM_PWM.h"

MotorParam g_st_motor_L, motor_R;
/*g st s i u8 u16 u32 s8 l m*/

/*******************************************************************************
* Function Name  : Timer3_PWMConfig. 
* Description    : TIM3 use for PWM .PWM波占空比为 IM_Pulse/TIM_Period.
									初始化设计PWM占空比为10%
* Input          : none.
* Return         : none.
*******************************************************************************/
void Timer3_PWMConfig(void)//Speed control Timer configuration
{	
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_OCInitTypeDef TIM_OCInitStructure;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);


		//Timer3 Config
		//PWM pin config
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		//Time Base configuration --18KHZ
		TIM_TimeBaseInitStructure.TIM_Prescaler = 5; // 
		TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStructure.TIM_Period = 1999; //
		TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);		

		//Configuration in PWM mode
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;     
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	
	  //TIM_OCInitStructure.TIM_Pulse = 0; 
	  //TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	  //TIM_OCInitStructure.TIM_Pulse = 0;
	  //TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_Pulse = 0;
		TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_Pulse = 0;
		TIM_OC4Init(TIM3, &TIM_OCInitStructure);
		//TIM3 counter enable
		TIM_Cmd(TIM3, ENABLE);

		//Main Output Enable
		TIM_CtrlPWMOutputs(TIM3, ENABLE);
}	

/*******************************************************************************
* Function Name  : MotorControl_Init.
* Description    : 初始化左轮和右轮对于JY01的方向控制IO口
* Input          : none.                  
* Return         : none.
*******************************************************************************/
void MotorControl_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		
	//	motor1.dir = 1;                 /*正方向*/
	//	motor1.inverse = 0;
	//	motor1.duty = 0.0;							/*占空比：初始值为0*/
	//	motor1.dutyRange = 2000;         /*占空比满量程值900*/
	//	motor1.dutyValue = 0;						/*当前占空比对应的量程*/
	//	motor1.workStatus = 0;					/*工作状态，当前值为停车*/
	//	motor1.CW_GPIOx = GPIOB;
	//	motor1.CW_GPIO_Pin = GPIO_Pin_11;
	//	motor1.TIMx = TIM3;
	//	motor1.CCRx = &(TIM3->CCR3);
		
		motor2.dir = 1;
	
#if(MOTOR_LEFT_OR_RIGHT==LEFT)
		motor2.inverse = 0;
#else
		motor2.inverse = 1;
#endif
		motor2.duty = 0.0;
		motor2.dutyRange = 2000;
		motor2.dutyValue = 0;
		motor2.workStatus = 0;
		motor2.CW_GPIOx = GPIOA;
		motor2.CW_GPIO_Pin = GPIO_Pin_7;
		motor2.TIMx = TIM3;
		motor2.CCRx = &(TIM3->CCR4);
	
//	motor3.dir = 1;
//	motor3.inverse = 0;
//	motor3.duty = 0.0;
//	motor3.dutyRange = 2000;
//	motor3.dutyValue = 0;
//	motor3.workStatus = 0;
//	motor3.CW_GPIOx = GPIOB;
//	motor3.CW_GPIO_Pin = GPIO_Pin_9;
//	motor3.TIMx = TIM3;
//	motor3.CCRx = &(TIM3->CCR1);
	
//	motor4.dir = 1;
//	motor4.inverse = 0;
//	motor4.duty = 0.0;
//	motor4.dutyRange = 2000;
//	motor4.dutyValue = 0;
//	motor4.workStatus = 0;
//	motor4.CW_GPIOx = GPIOB;
//	motor4.CW_GPIO_Pin = GPIO_Pin_8;
//	motor4.TIMx = TIM3;
//	motor4.CCRx = &(TIM3->CCR2);
	
		Timer3_PWMConfig();
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 |GPIO_Pin_9 |GPIO_Pin_10| GPIO_Pin_11;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
//	setMotorDir(&motor1, 1);
//	setMotorDir(&motor2, 1);
//	setMotorDir(&motor3, 1);
//	setMotorDir(&motor4, 1);
	
//	setMotorDirSpeed(&motor1, 0);
		setMotorDirSpeed(&motor2, 0);
//	setMotorDirSpeed(&motor3, 0);
//	setMotorDirSpeed(&motor4, 0);
}

//#define M1_FORWARD 0
//#define M1_REVERSE 1

void setMotorDir(MotorParam* motor, uint8_t dir)
{
		motor->dir = dir;
		if(motor->dir == 1)
		{
			GPIO_SetBits(motor->CW_GPIOx, motor->CW_GPIO_Pin);
		}
		else if(motor->dir == 0)
		{
			GPIO_ResetBits(motor->CW_GPIOx, motor->CW_GPIO_Pin);
		}
}

void setMotorSpeed(MotorParam* motor, uint16_t value)
{
		if(value >= 1999)
		{
				value = 1999;
				motor->workStatus = 1;
		}
		else if(value <= 50)
		{
				value = 0;
				motor->workStatus = 0;
		}
		
		motor->dutyValue = value;
		*(motor->CCRx) = value;
}

void setMotorDirSpeed(MotorParam* motor, int16_t speed_dir)
{
		if(speed_dir < 0)
		{
				if(motor->inverse == 0)
				{
					setMotorDir(motor, 0);
				}
				else if (motor->inverse == 1)
				{
					setMotorDir(motor, 1);
				}
				setMotorSpeed(motor, -speed_dir);
		}
		else
		{
				if(motor->inverse == 0)
				{
					setMotorDir(motor, 1);
				}
				else if (motor->inverse == 1)
				{
					setMotorDir(motor, 0);
				}
				setMotorSpeed(motor, speed_dir);		
		}
}


