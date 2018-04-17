#include "JTAG.h"
#include "INIT.h"
#include "can.h"
#include "delay.h"
#include "TIM.h"
#include "TIM_PWM.h"

void Init()
{
    JTAG_Set(SWD_ENABLE);
    CAN_GPIO_Config();				/* CAN�ܽų�ʼ�� */
    CAN_INIT();								/* CAN��ʼ��ģ�� */
    CAN_NVIC_Configuration(); /* ��������CAN���յ��ж����ȼ� */
    MotorControl_Init();
    TIM1_Int_Init(119,4999); 	/* 120Hz */
}
