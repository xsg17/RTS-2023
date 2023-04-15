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
//*                                               EXTERNS
//*********************************************************************************************************
//*/
//
//#ifdef   ACTION_MODULE
//#define  ACTION_EXT
//#else
//#define  ACTION_EXT  extern
//#endif
//#ifndef PI
//#define PI    3.14159265358979f
//#endif
//
///*
//*********************************************************************************************************
//*                                               DEFINES
//*********************************************************************************************************
//*/
//
//// Definition for ACTION resources 
//#define PORT_ACTION          GPIOB
//
//#define ACTION_PIN_RX          GPIO_Pin_11
//#define ACTION_PIN_TX          GPIO_Pin_10
//
//#define ACTION_AF_RX           GPIO_PinSource11
//#define ACTION_AF_TX           GPIO_PinSource10
//#define ACTION_AF_USART        GPIO_AF_USART3
//
//#define ACTION_USART                 USART3
//#define USART_ACTION_IRQHandler      USART3_IRQHandler
//#define USART_ACTION_IRQn            USART3_IRQn
//
//#define ACTION_PORT_ENABLE()  { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB,  ENABLE );\
//                                RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE ); }
//
//#define ACTION_BRAUDRATE          115200
//extern char flag_arbitraryPos;
///*
//*********************************************************************************************************
//*                                           FUNCTION PROTOTYPES
//*********************************************************************************************************
//*/
//
//void Action_Init (void);
//void Action_Send_Char(u8 Char);
//void Action_Reinit(void);
//void Action_AngleReinit(void);
//
///*
//*********************************************************************************************************
//*                                              MODULE END
//*********************************************************************************************************
//*/
