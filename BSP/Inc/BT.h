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
//*/      
//�ֱ�
#define UP         0X01
#define DOWN       0X02
#define LEFT       0X03
#define RIGHT      0X04
#define L_1        0X09
#define L_2        0X0B
#define R_1        0X0A
#define R_2        0X0C
#define TRIANGLE   0X05
#define SQUARE     0X07
#define CIRCLE     0X08
#define CROSS      0X06

//#define MODE       0X39
//#define R_Stick    0X3A
//#define L_Stick    0X3C

//���
#define RUP         0X0D
#define RDOWN       0X0E
#define RLEFT       0X0F
#define RRIGHT      0X10
#define RL_1        0X15
#define RL_2        0X17
#define RR_1        0X16
#define RR_2        0X18
#define RTRIANGLE   0X11
#define RSQUARE     0X13
#define RCIRCLE     0X14
#define RCROSS      0X12

//SELECT
#define SUP         0X21
#define SDOWN       0X22
#define SLEFT       0X23
#define SRIGHT      0X24
#define SL_1        0X29
#define SL_2        0X2B
#define SR_1        0X2A
#define SR_2        0X2C
#define STRIANGLE   0X25
#define SSQUARE     0X27
#define SCIRCLE     0X28
#define SCROSS      0X26

//���+SELECT
#define RSUP         0X2D
#define RSDOWN       0X2E
#define RSLEFT       0X2F
#define RSRIGHT      0X30
#define RSL_1        0X35
#define RSL_2        0X37
#define RSR_1        0X36
#define RSR_2        0X38
#define RSTRIANGLE   0X31
#define RSSQUARE     0X33
#define RSCIRCLE     0X34
#define RSCROSS      0X32


//********************************************************************************************************
//���ֱ�


//#define RockerR_Trans CH[0]
//#define RockerR_Longi CH[1]
//#define RockerL_Longi CH[2]
//#define RockerL_Trans CH[3]
//
//#define Pulley1       CH[4]
//#define Pulley2       CH[5]
//
//#define triggerL      CH[14]
//#define triggerR      CH[11]
//
//#define switchR       CH[13]
//#define seitchL       CH[12]
//
//#define button1       CH[6]              //从左到右 button6 无反应
//#define button2       CH[7] 
//#define button3       CH[8] 
//#define button4       CH[9] 
//#define button5       CH[10] 
//
// 
//
//#define RockerR_Trans_MEM    CH_MEM[0]
//#define RockerR_Longi_MEM    CH_MEM[1]
//#define RockerL_Longi_MEM    CH_MEM[2]
//#define RockerL_Trans_MEM    CH_MEM[3]
//                               
//#define Pulley1_MEM          CH_MEM[4]
//#define Pulley2_MEM          CH_MEM[5]
//                              
//#define triggerL_MEM         CH_MEM[14]
//#define triggerR_MEM         CH_MEM[11]
//                               
//#define switchR_MEM          CH_MEM[13]
//#define switchL_MEM          CH_MEM[12]
//
//#define button1_MEM       CH_MEM[6]             
//#define button2_MEM       CH_MEM[7] 
//#define button3_MEM       CH_MEM[8] 
//#define button4_MEM       CH_MEM[9] 
//#define button5_MEM       CH_MEM[10]
                       

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
//#define button6       CH[11] 

 

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
//#define button6_MEM       CH_MEM[11]



//********************************************************************************************************


//********************************************************************************************************

#define     BT_USART		USART3
//
//#define     BT_TX_Enable       { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE ); }
//#define     BT_RX_Enable       { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE ); }
//#define     BT_Enable	       { RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE ); }
//
//#define     GPIO_TX             GPIOA
//#define     PIN_TX              GPIO_Pin_2
//#define     PINSor_TX		GPIO_PinSource2
//
//#define     GPIO_RX	        GPIOA
//#define     PIN_RX              GPIO_Pin_3
//#define     PINSor_RX           GPIO_PinSource3
//
//#define     GPIO_AF_BT	        GPIO_AF_USART2
//
//#define     BT_USART_IRQn               UART4_IRQn
//#define     BT_USART_IRQHandler         UART4_IRQHandler
//#define     USART_BOUND                 115200

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

//void BT_Init( void );
void BT_Send_Char( u8 Char );
void send_oled( void );
//void usart1_init()��
//void TestBt_Send_Float( float senddata );

/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/

extern u8 BT_Step;
extern u8 KEY_VALUE;

extern float USE_TIME; // �����ʱ����