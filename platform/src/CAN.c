#include "stm32f10x.h"
#include "CAN.h"
#include "stdio.h"
#include "TIM.h"
#include "delay.h"
#include "VERSION.h"
#include "TIM_PWM.h"

s16 AimSpeed_L = 0;
s16 AimSpeed_R = 0;
uint8_t receive_speed = 0;
s16 AimSpeed = 0;

u8 P = 0,I = 0,D = 0;
u8 version_canformat[8] = {0x13,0x01,0x00,0x00,0x00,0x00,0x00,0x00};

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

uint32_t JumpAddress;
typedef  void (*pFunction)(void);
pFunction Jump_To_Application;



/* 在中断处理函数中返回 */
__IO uint32_t ret = 0;

volatile TestStatus TestRx;	

/*******************************************************************************
* Function Name  : CAN_NVIC_Configuration
* Description    : CAN RX0 中断优先级配置
* Input          : None.
* Return         : none.
*******************************************************************************/
void CAN_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

  	/* Configure the NVIC Preemption Priority Bits */  
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	  /* enabling interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel=USB_LP_CAN1_RX0_IRQn;;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : CAN_GPIO_Config
* Description    : CAN GPIO 和时钟配置
* Input          : None.
* Return         : none.
*******************************************************************************/
void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 
	
    /* 复用功能和GPIOB端口时钟使能 */ 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);	    	                    											   
  
	  /* CAN1 模块时钟使能 */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);   

    /* Configure CAN pin: RX , PB8, 上拉输入 */	 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
  
    /* Configure CAN pin: TX , PB9, 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
}

/*******************************************************************************
* Function Name  : CAN_INIT
* Description    : CAN初始化
* Input          : None.
* Return         : none.
*******************************************************************************/
void CAN_INIT(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
 	  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
 
	  /* 使能PORTA, CAN1时钟 */
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);                   											 
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	/* 复用推挽*/
	  GPIO_Init(GPIOA, &GPIO_InitStructure);		
 
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  /* 上拉输入 */
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	  /* CAN单元设置 */
   	CAN_InitStructure.CAN_TTCM=DISABLE;						 /* 非时间触发通信模式 */
 	  CAN_InitStructure.CAN_ABOM=DISABLE;						 /* 软件自动离线管理	*/
	  CAN_InitStructure.CAN_AWUM=DISABLE;						 /* 睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位) */
	  CAN_InitStructure.CAN_NART=ENABLE;						 /* 禁止报文自动传送 */
	  CAN_InitStructure.CAN_RFLM=DISABLE;						 /* 报文不锁定,新的覆盖旧的 */ 
	  CAN_InitStructure.CAN_TXFP=DISABLE;						 /* 优先级由报文标识符决定 */
	  CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	         /* 模式设置： mode:0,普通模式;1,回环模式; */
	  /* 设置波特率 */
   	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;				/* 重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq */
  	CAN_InitStructure.CAN_BS1=CAN_BS1_13tq;       /* Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq */
	  CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;        /* Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq */
	  CAN_InitStructure.CAN_Prescaler=18;           /* 分频系数(Fdiv)为brp+1 */
	  CAN_Init(CAN1, &CAN_InitStructure);           /* 初始化CAN1 */

    /* CAN过滤器初始化 */
    CAN_FilterInitStructure.CAN_FilterNumber=1;   /* 指定过滤器为1 */
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;    /* 指定过滤器为标识符屏蔽位模式 */
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   /* 过滤器位宽为32位 */
    CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)CAN_ID_REC_ALLOW<<21)&0xffff0000)>>16;    /* 32位ID */
    CAN_FilterInitStructure.CAN_FilterIdLow=(((u32)CAN_ID_REC_ALLOW<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF;    /* 过滤器屏蔽标识符的高16位值 */
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;     /* 过滤器屏蔽标识符的低16位值 */
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    /* 设定了指向过滤器的FIFO为0 */
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;    /* 使能过滤器 */
    CAN_FilterInit(&CAN_FilterInitStructure);    /*	按上面的参数初始化过滤器 */

    CAN_FilterInit(&CAN_FilterInitStructure);    /* 滤波器初始化 */
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);       /* FIFO0消息挂号中断允许 */		
}  

 
uint32_t FLASH_PagesMask(__IO uint32_t Size)
{
    uint32_t pagenumber = 0x0;
    uint32_t size = Size;

    if ((size % PAGE_SIZE) != 0)
    {
      pagenumber = (size / PAGE_SIZE) + 1;
    }
    else
    {
      pagenumber = size / PAGE_SIZE;
    }
    return pagenumber;
}
 
FLASH_Status EraseAllNeedPage(uint32_t startAddress, uint32_t fileSize)
{
	  uint8_t NbrOfPage = 0;
	  uint8_t EraseCounter = 0x0;
	  FLASH_Status FLASHStatus = FLASH_COMPLETE;
	  NbrOfPage = FLASH_PagesMask(fileSize);
	  /* Erase the FLASH pages */
	  for (EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	  {
	  	FLASHStatus = FLASH_ErasePage(startAddress + (PAGE_SIZE * EraseCounter));
	  }	
	  return FLASHStatus;
}

/*******************************************************************************
* Function Name  : ReadIapMessage
* Description    : 读取Iap的信息，判断状态
* Input          : None.
* Return         : none.
*******************************************************************************/
uint32_t ReadIapMessage(void)
{ 
	  uint32_t destAddr = MessageSavedAddress;
	  return (*((uint32_t *)destAddr));
}

/*******************************************************************************
* Function Name  : WriteIapMessage
* Description    : 在IAP信息存储区写入一些信息
* Input          : messageSaved
* Return         : 
*******************************************************************************/
uint8_t WriteIapMessage(uint32_t messageSaved)
{
	  uint32_t destAddr = MessageSavedAddress;
		/* 记录文件大小信息 */
	  uint32_t fileSize = *((uint32_t *)(destAddr+4)); 
	  EraseAllNeedPage(destAddr, PAGE_SIZE);
	  FLASH_ProgramWord(destAddr, messageSaved);
	  if(*((uint32_t*)destAddr) != messageSaved)
	  {
	  	return 1;
	  }
		/* 恢复filesize信息 */	
	  FLASH_ProgramWord((destAddr+4), fileSize);
	  if(*((uint32_t *)(destAddr+4)) != fileSize)
	  {
	  	return 2;
	  }
	  return 0;
}

/*******************************************************************************
* Function Name  : ReadLastFileSize
* Description    : 读取文件大小
* Input          : None.
* Return         : destAddr
*******************************************************************************/
uint32_t ReadLastFileSize(void)
{
	  uint32_t destAddr = MessageSavedAddress+4;
	  return (*((uint32_t *)destAddr));
}

/*******************************************************************************
* Function Name  : WriteLastFileSize
* Description    : 写入文件大小信息
* Input          : fileSize
* Return         : 
*******************************************************************************/
uint8_t WriteLastFileSize(uint32_t fileSize)
{
	  uint32_t destAddr = MessageSavedAddress;
		/* 记录messageSaved */
	  uint32_t messageTempSaved = *((uint32_t *)(destAddr)); 
	  EraseAllNeedPage(destAddr, PAGE_SIZE);
	
	  FLASH_ProgramWord((destAddr+4), fileSize);
	  if(*((uint32_t*)(destAddr+4)) != fileSize)
	  {
	  	return 1;
	  }
		/* 恢复messageSaved */
	  FLASH_ProgramWord(destAddr, messageTempSaved);
	  if(*((uint32_t *)(destAddr)) != messageTempSaved)
	  {
	  	return 2;
	  }
	  return 0;
}


__asm void SystemReset(void)
{
    MOV R0, #1            
    MSR FAULTMASK, R0    
    LDR R0, =0xE000ED0C  
    LDR R1, =0x05FA0004  
    STR R1, [R0]           
 
deadloop
    B deadloop        
}


void IAPRun(void)
{
	  /* Test if user code is programmed starting from address "ApplicationAddress" */
	  if (((*(__IO uint32_t*)IAPAddress) & 0x2FFE0000 ) == 0x20000000)
	  { 
		    /* Jump to user application */
		    JumpAddress = *(__IO uint32_t*) (IAPAddress + 4);
		    Jump_To_Application = (pFunction) JumpAddress;
		    /* Initialize user application's Stack Pointer */
		    __set_MSP(*(__IO uint32_t*) IAPAddress);
		    Jump_To_Application();
	}
}
 
 
 
u32 canSendSpeedError=0;
/*******************************************************************************
* Function Name  : can_tx
* Description    : 发送两个字节的数据
* Input          : data[]
* Return         : none
*******************************************************************************/
void can_tx(u8 data[])
{ 
	  u16 i;
    CanTxMsg TxMessage;  

    TxMessage.StdId=SELF_STD_CAN_ID;	/* 标准标识符为0x0002 */
    TxMessage.ExtId=0x0000;           /* 扩展标识符0x0000 */
    TxMessage.IDE=CAN_ID_STD;         /* 使用标准标识符 */
    TxMessage.RTR=CAN_RTR_DATA;       /* 为数据帧 */
    TxMessage.DLC=8;	                /* 消息的数据长度为2个字节 */
	
	  for(i=0; i<8; i++)
	  	TxMessage.Data[i] = data[i]; 

    CAN_Transmit(CAN1,&TxMessage);    /* 发送数据 */ 
}


CanRxMsg RxMessage;

/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : USB中断和CAN接收中断服务程序，USB跟CAN公用I/O，这里只用到CAN的中断。
* Input          : none
* Return         : none
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    u8 i;
 
    RxMessage.StdId=0x00;
    RxMessage.ExtId=0x00;
    RxMessage.IDE=0;
    RxMessage.DLC=0;
    RxMessage.FMI=0;
	  for(i=0; i<8; i++)
		    RxMessage.Data[i] = 0x00;   

    CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);    /* 接收FIFO0中的数据 */ 
	
		/* 收到的是速度信息 */
	  if((RxMessage.StdId == CAN_ID_REC_ALLOW) && (RxMessage.Data[0]==0x01)) 
	  {
#if(MOTOR_LEFT_OR_RIGHT==LEFT)
		    /* 左轮速度 */
			/*gs16_aimSpeedL*/
		    AimSpeed_L = (RxMessage.Data[1] << 8) |(RxMessage.Data[2]);
		    AimSpeed = AimSpeed_L;
#else
		    /* 右轮速度 */
		    AimSpeed_R = (RxMessage.Data[3] << 8) |(RxMessage.Data[4]);
		    AimSpeed = AimSpeed_R;
#endif		
		    receive_speed = 1;
	  }
	  else if((RxMessage.StdId == CAN_ID_REC_ALLOW) && (RxMessage.Data[0]==0x00) && (RxMessage.Data[1]==0x13))
	  {
		    version_canformat[7] = BCC_CheckSum(version_canformat,7);
		    can_tx(version_canformat);
	  }	
} 
