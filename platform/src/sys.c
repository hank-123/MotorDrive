#include "stm32f10x.h"
#include "sys.h"

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
	
		/*	Right wheel encoder interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;			
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
	  NVIC_Init(&NVIC_InitStructure); 
	
		/*	Left wheel encoder interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;			
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;					
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
	  NVIC_Init(&NVIC_InitStructure); 
	
		/*	Handle timer interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	  NVIC_Init(&NVIC_InitStructure);  
}
