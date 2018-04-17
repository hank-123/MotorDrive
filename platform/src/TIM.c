/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "TIM.h"
#include "math.h"
#include "can.h"
#include "VERSION.h"
#include "stm32f10x_flash.h"
#include "TIM_PWM.h"

extern int AimSpeed_L;
extern int AimSpeed_R;
extern s16 AimSpeed;
extern uint8_t receive_speed;
extern u8 datasend[8];
MotorParam motor1, motor2, motor3, motor4;

/*******************************************************************************
* Function Name  : TIM1_Int_Init.
* Description    : TIM1 use for 100ms time counter
* Input          : @para arr: 定时器自动重装载值.
                   @para psc: 定时器预分频器
* Return         : none.
*******************************************************************************/
void TIM1_Int_Init(u16 arr,u16 psc)
{

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

    TIM_TimeBaseStructure.TIM_Period = arr; 
    TIM_TimeBaseStructure.TIM_Prescaler =(psc-1);
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);

    TIM_ITConfig(      
      TIM1,            
      TIM_IT_Update  |
      TIM_IT_Trigger,  
      ENABLE  	     
    );
	

		TIM_Cmd(TIM1, ENABLE);	 
}

/*******************************************************************************
* Function Name  : TIM1_IRQHandler
* Description    : 定时器1的中断服务程序
* Input          : None.
* Return         : none.
*******************************************************************************/
void TIM1_UP_IRQHandler(void)   //TIM4中断
{
	  static uint8_t _10Hz = 0;
	  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)  //检查TIM1更新中断发生与否
		{
		  	_10Hz++;
		  	TIM_ClearITPendingBit(TIM1, TIM_IT_Update  );  //清除TIMx更新中断标志 

			  if (_10Hz == 12)
			  {
			    	_10Hz = 0;
			 	if(receive_speed == 1)
				{
//					setMotorDirSpeed(&motor1, AimSpeed_L);
				  	setMotorDirSpeed(&motor2, AimSpeed);
				}
				else
				{
//					setMotorDirSpeed(&motor1, 70);
//					setMotorDirSpeed(&motor2, 70);
				}
				receive_speed = 0;
			}
	}
}

//异或校验
u8 BCC_CheckSum(u8 *buf,u8 len)
{
	  u8 i;
	  u8 checksum = 0;
	  for(i=0; i<len; i++)
	  {
		  checksum ^= *buf++;
  	}
	  return checksum;
}
