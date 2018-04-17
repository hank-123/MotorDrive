#ifndef __CAN_H
#define	__CAN_H

#include "stm32f10x.h" 

#define MessageSavedAddress 0x0801F800
#define ApplicationAddress  0x08004000 /* Flash user program offset    0x4000-----16KB */
#define CopyAreaAddress     0x08011C00  /* 请注意FLASH大小为128KB,考虑到BOOTLOADER，一个保存当前程序的区域，一个备份区域，故程序大小不得超过55KB */
#define ENTER_IAP  0x0500  
#define ENTER_APP  0x0A00  
#define IAP_SUCCESS 0x8000
#define IAP_FAIL    0x4000
#define APP_FIT_COPY 	0x2000
#define APP_NOT_FIT_COPY  0x1000
#define PAGE_SIZE 1024
#define SELF_STD_CAN_ID 0x02
#define CAN_ID_REC_ALLOW 0x08
#define IAPAddress 0x08000000 /* Flash user program offset    0x3000-----12KB*/

extern uint8_t receive_speed;
extern u8 datasend[8];
extern s16 AimSpeed;

void CAN_INIT(void);
void can_tx(u8 data[]);
void can_rx(void);
void CAN_NVIC_Configuration(void);
void CAN_GPIO_Config(void);

uint32_t ReadIapMessage(void);
uint8_t WriteIapMessage(uint32_t messageSaved);
uint8_t WriteLastFileSize(uint32_t fileSize);
uint32_t ReadLastFileSize(void);


#endif /* __CAN_H */
