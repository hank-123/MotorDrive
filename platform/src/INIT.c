#include "JTAG.h"
#include "INIT.h"
#include "can.h"
#include "delay.h"
#include "TIM.h"
#include "TIM_PWM.h"

void Init()
{
    JTAG_Set(SWD_ENABLE);
    CAN_GPIO_Config();				/* CAN管脚初始化 */
    CAN_INIT();								/* CAN初始化模块 */
    CAN_NVIC_Configuration(); /* 单独设置CAN接收的中断优先级 */
    MotorControl_Init();
    TIM1_Int_Init(119,4999); 	/* 120Hz */
}
