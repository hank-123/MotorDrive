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



/* ���жϴ������з��� */
__IO uint32_t ret = 0;

volatile TestStatus TestRx;	

/*******************************************************************************
* Function Name  : CAN_NVIC_Configuration
* Description    : CAN RX0 �ж����ȼ�����
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
* Description    : CAN GPIO ��ʱ������
* Input          : None.
* Return         : none.
*******************************************************************************/
void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 
	
    /* ���ù��ܺ�GPIOB�˿�ʱ��ʹ�� */ 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);	    	                    											   
  
	  /* CAN1 ģ��ʱ��ʹ�� */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);   

    /* Configure CAN pin: RX , PB8, �������� */	 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
  
    /* Configure CAN pin: TX , PB9, ����������� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
}

/*******************************************************************************
* Function Name  : CAN_INIT
* Description    : CAN��ʼ��
* Input          : None.
* Return         : none.
*******************************************************************************/
void CAN_INIT(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
 	  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
 
	  /* ʹ��PORTA, CAN1ʱ�� */
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);                   											 
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	/* ��������*/
	  GPIO_Init(GPIOA, &GPIO_InitStructure);		
 
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  /* �������� */
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	  /* CAN��Ԫ���� */
   	CAN_InitStructure.CAN_TTCM=DISABLE;						 /* ��ʱ�䴥��ͨ��ģʽ */
 	  CAN_InitStructure.CAN_ABOM=DISABLE;						 /* ����Զ����߹���	*/
	  CAN_InitStructure.CAN_AWUM=DISABLE;						 /* ˯��ģʽͨ���������(���CAN->MCR��SLEEPλ) */
	  CAN_InitStructure.CAN_NART=ENABLE;						 /* ��ֹ�����Զ����� */
	  CAN_InitStructure.CAN_RFLM=DISABLE;						 /* ���Ĳ�����,�µĸ��Ǿɵ� */ 
	  CAN_InitStructure.CAN_TXFP=DISABLE;						 /* ���ȼ��ɱ��ı�ʶ������ */
	  CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	         /* ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; */
	  /* ���ò����� */
   	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;				/* ����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq */
  	CAN_InitStructure.CAN_BS1=CAN_BS1_13tq;       /* Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq */
	  CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;        /* Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq */
	  CAN_InitStructure.CAN_Prescaler=18;           /* ��Ƶϵ��(Fdiv)Ϊbrp+1 */
	  CAN_Init(CAN1, &CAN_InitStructure);           /* ��ʼ��CAN1 */

    /* CAN��������ʼ�� */
    CAN_FilterInitStructure.CAN_FilterNumber=1;   /* ָ��������Ϊ1 */
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;    /* ָ��������Ϊ��ʶ������λģʽ */
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   /* ������λ��Ϊ32λ */
    CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)CAN_ID_REC_ALLOW<<21)&0xffff0000)>>16;    /* 32λID */
    CAN_FilterInitStructure.CAN_FilterIdLow=(((u32)CAN_ID_REC_ALLOW<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF;    /* ���������α�ʶ���ĸ�16λֵ */
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;     /* ���������α�ʶ���ĵ�16λֵ */
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    /* �趨��ָ���������FIFOΪ0 */
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;    /* ʹ�ܹ����� */
    CAN_FilterInit(&CAN_FilterInitStructure);    /*	������Ĳ�����ʼ�������� */

    CAN_FilterInit(&CAN_FilterInitStructure);    /* �˲�����ʼ�� */
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);       /* FIFO0��Ϣ�Һ��ж����� */		
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
* Description    : ��ȡIap����Ϣ���ж�״̬
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
* Description    : ��IAP��Ϣ�洢��д��һЩ��Ϣ
* Input          : messageSaved
* Return         : 
*******************************************************************************/
uint8_t WriteIapMessage(uint32_t messageSaved)
{
	  uint32_t destAddr = MessageSavedAddress;
		/* ��¼�ļ���С��Ϣ */
	  uint32_t fileSize = *((uint32_t *)(destAddr+4)); 
	  EraseAllNeedPage(destAddr, PAGE_SIZE);
	  FLASH_ProgramWord(destAddr, messageSaved);
	  if(*((uint32_t*)destAddr) != messageSaved)
	  {
	  	return 1;
	  }
		/* �ָ�filesize��Ϣ */	
	  FLASH_ProgramWord((destAddr+4), fileSize);
	  if(*((uint32_t *)(destAddr+4)) != fileSize)
	  {
	  	return 2;
	  }
	  return 0;
}

/*******************************************************************************
* Function Name  : ReadLastFileSize
* Description    : ��ȡ�ļ���С
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
* Description    : д���ļ���С��Ϣ
* Input          : fileSize
* Return         : 
*******************************************************************************/
uint8_t WriteLastFileSize(uint32_t fileSize)
{
	  uint32_t destAddr = MessageSavedAddress;
		/* ��¼messageSaved */
	  uint32_t messageTempSaved = *((uint32_t *)(destAddr)); 
	  EraseAllNeedPage(destAddr, PAGE_SIZE);
	
	  FLASH_ProgramWord((destAddr+4), fileSize);
	  if(*((uint32_t*)(destAddr+4)) != fileSize)
	  {
	  	return 1;
	  }
		/* �ָ�messageSaved */
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
* Description    : ���������ֽڵ�����
* Input          : data[]
* Return         : none
*******************************************************************************/
void can_tx(u8 data[])
{ 
	  u16 i;
    CanTxMsg TxMessage;  

    TxMessage.StdId=SELF_STD_CAN_ID;	/* ��׼��ʶ��Ϊ0x0002 */
    TxMessage.ExtId=0x0000;           /* ��չ��ʶ��0x0000 */
    TxMessage.IDE=CAN_ID_STD;         /* ʹ�ñ�׼��ʶ�� */
    TxMessage.RTR=CAN_RTR_DATA;       /* Ϊ����֡ */
    TxMessage.DLC=8;	                /* ��Ϣ�����ݳ���Ϊ2���ֽ� */
	
	  for(i=0; i<8; i++)
	  	TxMessage.Data[i] = data[i]; 

    CAN_Transmit(CAN1,&TxMessage);    /* �������� */ 
}


CanRxMsg RxMessage;

/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : USB�жϺ�CAN�����жϷ������USB��CAN����I/O������ֻ�õ�CAN���жϡ�
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

    CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);    /* ����FIFO0�е����� */ 
	
		/* �յ������ٶ���Ϣ */
	  if((RxMessage.StdId == CAN_ID_REC_ALLOW) && (RxMessage.Data[0]==0x01)) 
	  {
#if(MOTOR_LEFT_OR_RIGHT==LEFT)
		    /* �����ٶ� */
			/*gs16_aimSpeedL*/
		    AimSpeed_L = (RxMessage.Data[1] << 8) |(RxMessage.Data[2]);
		    AimSpeed = AimSpeed_L;
#else
		    /* �����ٶ� */
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
