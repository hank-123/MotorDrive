#ifndef _TIM_H
#define _TIM_H
#include "stm32f10x.h"


extern s32 counter_L;
extern s32 counter_R;
extern u8 ZF_L;
extern u8 ZF_R;
extern u8 Dir_R;
extern u8 Dir_L;

void TIM1_Int_Init(u16 arr,u16 psc);
u8 BCC_CheckSum(u8 *buf,u8 len);

#endif
