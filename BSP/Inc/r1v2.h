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

#ifdef   R1V2_MODULE
#define  R1V2_EXT
#else
#define  R1V2_EXT  extern
#endif
#ifndef PI
#define PI    3.14159265358979f
#endif

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

// Definition for R1V2 resources 
#define PORT_R1V2          GPIOB

#define R1V2_PIN_RX          GPIO_Pin_7
#define R1V2_PIN_TX          GPIO_Pin_6

#define R1V2_AF_RX           GPIO_PinSource7
#define R1V2_AF_TX           GPIO_PinSource6
#define R1V2_AF_USART        GPIO_AF_USART1

#define R1V2_USART                 USART1
#define USART_R1V2_IRQHandler      USART1_IRQHandler
#define USART_R1V2_IRQn            USART1_IRQn

#define R1V2_PORT_ENABLE()  { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB,  ENABLE );\
                               RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE ); }

#define R1V2_BRAUDRATE          100000
/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void SBUS_init (void);
void USART1_IRQHandler(void);
void R1V2_Data_Count(uint8_t *buf);

extern uint8_t R1V2_RX_BUF[26];
extern uint16_t CH[18];  // 通道值
extern uint16_t CH_MEM[18];
extern uint8_t  rc_flag;

typedef struct
{
	uint16_t CH1;//通道1数值
	uint16_t CH2;//通道2数值
	uint16_t CH3;//通道3数值
	uint16_t CH4;//通道4数值
	uint16_t CH5;//通道5数值
	uint16_t CH6;//通道6数值
    uint16_t CH7;//通道7数值
    uint16_t CH8;//通道8数值
    uint16_t CH9;//通道9数值
    uint16_t CH10;//通道10数值
    uint16_t CH11;//通道11数值
    uint16_t CH12;//通道12数值
    uint16_t CH13;//通道13数值
    uint16_t CH14;//通道14数值
    uint16_t CH15;//通道15数值
    uint16_t CH16;//通道16数值
	uint8_t ConnectState;//遥控器与接收器连接状态 0=未连接，1=正常连接
}SBUS_CH;
/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/
