/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "includes.h"

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define   bound     115200
extern u8 ReceiveData, ReceiveData_2;

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

// 串口 USART3
void usart_init(void);
void send_uchar(unsigned char udata);
void send_char(char data);
// 发送给上位机作图  电机PID
void SendToDraw( uint8_t data8_H1, uint8_t data8_L1, 
                 uint8_t data8_H2, uint8_t data8_L2, 
                 uint8_t data8_H3, uint8_t data8_L3,  
                 uint8_t data8_H4, uint8_t data8_L4 ); 

// 串口 USART6
void usart_init_draw(void);
void send_uchar_draw(unsigned char udata);
// 发送给上位机作图  底盘PID
//void SendToDraw2( uint8_t data8_H1, uint8_t data8_L1);
void SendToDraw2( uint8_t data8_H1, uint8_t data8_L1, 
                  uint8_t data8_H2, uint8_t data8_L2 ,
                  uint8_t data8_H3,uint8_t data8_L3,
                  uint8_t data8_H4,uint8_t data8_L4); // 传递给上位机画图

/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/