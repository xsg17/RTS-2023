#include "includes.h"

#define PORT_AUX    GPIOF
#define PIN_AUX    GPIO_Pin_6
#define PORT_MD0    GPIOC
#define PIN_MD0    GPIO_Pin_0

#define AUX_Toggle()    GPIO_ToggleBits(PORT_AUX, PIN_AUX)
#define AUX_ON()        GPIO_SetBits(PORT_AUX, PIN_AUX)
#define AUX_OFF()       GPIO_ResetBits(PORT_AUX, PIN_AUX)

#define MD0_Toggle()    GPIO_ToggleBits(PORT_MD0, PIN_MD0)
#define MD0_ON()        GPIO_SetBits(PORT_MD0, PIN_MD0)
#define MD0_OFF()       GPIO_ResetBits(PORT_MD0, PIN_MD0)

//#define USART3_MAX_SEND_LEN		600					//����ͻ����ֽ���

void LoRa_Init(void);
//void LoRa_ReceData(void);
//void LoRa_SendData(void);
void BT_usart3_init();				//����3��ʼ�� 
//void u3_printf(char* fmt, ...);

///*
//*********************************************************************************************************
//* ʹ�õĽӿڣ�
//*             PC12 -> TX 
//*             PD2  -> RX 
//*             PC11 -> BT_Status
//* ʹ�õ����裺
//*             UART5
//*********************************************************************************************************
//*/
//
///*
//*********************************************************************************************************
//*                                             INCLUDE FILES
//*********************************************************************************************************
//*/
//
//#pragma once
//#include "includes.h"
//
///*
//*********************************************************************************************************
//*                                               DEFINES
//*********************************************************************************************************



//********************************************************************************************************



//********************************************************************************************************
//Corresponding key position for old version remote control -----> Jumper T-pro                       
//********************************************************************************************************

/*
#define RockerR_Horizontal CH[0]
#define RockerR_Vertical CH[1]
#define RockerL_Vertical CH[2]
#define RockerL_Horizontal CH[3]

#define Pulley1       CH[4]
#define Pulley2       CH[5]

#define triggerL      CH[14]
#define triggerR      CH[11]

#define switchR       CH[13]
#define switchL       CH[12]

#define button1       CH[6]              //从左到右 button6 无反应
#define button2       CH[7] 
#define button3       CH[8] 
#define button4       CH[9] 
#define button5       CH[10] 


 

#define RockerR_Horizontal_MEM    CH_MEM[0]
#define RockerR_Vertical_MEM    CH_MEM[1]
#define RockerL_Vertical_MEM    CH_MEM[2]
#define RockerL_Horizontal_MEM    CH_MEM[3]
                               
#define Pulley1_MEM          CH_MEM[4]
#define Pulley2_MEM          CH_MEM[5]
                              
#define triggerL_MEM         CH_MEM[14]
#define triggerR_MEM         CH_MEM[11]
                               
#define switchR_MEM          CH_MEM[13]
#define switchL_MEM          CH_MEM[12]

#define button1_MEM       CH_MEM[6]             
#define button2_MEM       CH_MEM[7] 
#define button3_MEM       CH_MEM[8] 
#define button4_MEM       CH_MEM[9] 
#define button5_MEM       CH_MEM[10]
*/

//********************************************************************************************************
//Corresponding key position for WFLY-ET16S
//********************************************************************************************************

//ROCCKER      353-1695
#define RockerR_Horizontal CH[0]
#define RockerR_Vertical   CH[1]
#define RockerL_Vertical   CH[2]
#define RockerL_Horizontal CH[3]

//SWITCH       353-1024-1695
#define SA                 CH[4]
#define SB                 CH[5]
#define SC                 CH[6]
#define SD                 CH[7]
#define SE                 CH[8]
#define SF                 CH[9]                           //only have 2 gear
#define SG                 CH[15]
#define SH                 CH[12]                          //only have 2 gear
                           
//KNOB                     
#define LD                 CH[10]
#define RD                 CH[13]
                           
//ROLLER                   
#define LS                 CH[14]
#define RS                 CH[11]


//********************************************************************************************************
// memery
//********************************************************************************************************

#define RockerR_Horizontal_MEM CH_MEM[0]
#define RockerR_Vertical_MEM   CH_MEM[1]
#define RockerL_Vertical_MEM   CH_MEM[2]
#define RockerL_Horizontal_MEM CH_MEM[3]
#define SA_MEM                 CH_MEM[4]
#define SB_MEM                 CH_MEM[5]
#define SC_MEM                 CH_MEM[6]
#define SD_MEM                 CH_MEM[7]
#define SE_MEM                 CH_MEM[8]
#define SF_MEM                 CH_MEM[9]
#define SG_MEM                 CH_MEM[15]
#define SH_MEM                 CH_MEM[12]                     
#define LD_MEM                 CH_MEM[10]
#define RD_MEM                 CH_MEM[13]                  
#define LS_MEM                 CH_MEM[14]
#define RS_MEM                 CH_MEM[11]


//********************************************************************************************************


//********************************************************************************************************

#define     BT_USART		USART3

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/


void BT_Send_Char( u8 Char );
void send_oled( void );


/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/

extern u8 BT_Step;
extern u8 KEY_VALUE;

extern float USE_TIME;