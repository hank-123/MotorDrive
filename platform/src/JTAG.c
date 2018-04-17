#include "JTAG.h"

void JTAG_Set(u8 mode)
{
	  u32 temp;
	  temp=mode;
	  temp<<=25;
	  RCC->APB2ENR|=1<<0;       
	  AFIO->MAPR&=0XF8FFFFFF; 
	  AFIO->MAPR|=temp;       
} 
