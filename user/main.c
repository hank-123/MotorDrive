/***********************************************************
Copyright(C), 2018 - 2028, 普渡科技
File name: MotorDrive
Author:  zsd  Version:  v1.0  Date:  2018.04.17
Description:  电机驱动    
Others:       无
Function List:  
History:
    <author>    <time>    <version>    <desc>
***********************************************************/

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_adc.h"
#include <stdio.h>
#include <math.h>
#include "INIT.h"
#include "sys.h"

int main(void)
{
	  Init();
	  NVIC_Configuration();

	  while(1)
		{
			
	  }
}




