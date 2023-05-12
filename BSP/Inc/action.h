/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "includes.h"

/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/

#ifdef   ACTION_MODULE
#define  ACTION_EXT
#else
#define  ACTION_EXT  extern
#endif
#ifndef PI
#define PI    3.14159265358979f
#endif

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

// Definition for ACTION resources 
#define PORT_ACTION            GPIOE
#define ACTION_PIN_RX          GPIO_Pin_0
#define ACTION_PIN_TX          GPIO_Pin_1

#define ACTION_AF_RX           GPIO_PinSource0
#define ACTION_AF_TX           GPIO_PinSource1
#define ACTION_AF_USART        GPIO_AF_UART8

#define ACTION_USART                 UART8
#define USART_ACTION_IRQHandler      UART8_IRQHandler
#define USART_ACTION_IRQn            UART8_IRQn

#define ACTION_PORT_ENABLE()  { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE,  ENABLE );\
                                RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART8, ENABLE ); }

#define ACTION_BRAUDRATE          115200
extern char flag_arbitraryPos;
/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void Action_Init (void);
void Action_Send_Char(u8 Char);
void Action_Reinit(void);
void Action_AngleReinit(void);

/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/
