#define  R1V2_MODULE
#include "r1v2.h"

uint8_t R1V2_RX_BUF[26];
uint16_t CH[18];  // 通道值
uint16_t CH_MEM[18];
uint8_t rc_flag = 0;
void SBUS_init(void)
{
 GPIO_InitTypeDef GPIO_InitStruct;
 USART_InitTypeDef USART_InitStruct;
 NVIC_InitTypeDef NVIC_InitStruct;
 
 // Peripheral Clock Enable
 R1V2_PORT_ENABLE();
 
 // USART GPIO configuration 
 GPIO_PinAFConfig( PORT_R1V2, R1V2_AF_TX, R1V2_AF_USART );
 GPIO_PinAFConfig( PORT_R1V2, R1V2_AF_RX, R1V2_AF_USART );
 
 GPIO_InitStruct.GPIO_Pin = R1V2_PIN_TX | R1V2_PIN_RX;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
 GPIO_Init( PORT_R1V2, &GPIO_InitStruct );
 
 // Enable the USART OverSampling by 8 
 USART_OverSampling8Cmd( R1V2_USART, ENABLE );
 
 // USART configuration 
 USART_InitStruct.USART_BaudRate = R1V2_BRAUDRATE;
 USART_InitStruct.USART_WordLength = USART_WordLength_8b;
 USART_InitStruct.USART_StopBits = USART_StopBits_2;
 USART_InitStruct.USART_Parity = USART_Parity_Even;
 USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 USART_Init( R1V2_USART, &USART_InitStruct );
 
 NVIC_InitStruct.NVIC_IRQChannel = USART_R1V2_IRQn;
 NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
 NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
 NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init( &NVIC_InitStruct );
 
 // Enable USART 
 USART_Cmd(R1V2_USART, ENABLE);
 USART_ITConfig( R1V2_USART, USART_IT_RXNE, ENABLE );
 
 //BEEP_Init(2000);
}
 uint8_t Rx_Sta = 0;
void USART1_IRQHandler(void)
{
  CPU_SR_ALLOC();
  OS_ERR err;
  CPU_CRITICAL_ENTER();
  OSIntEnter(); // Tell uC/OS-III that we are starting an ISR
  CPU_CRITICAL_EXIT();
    uint8_t res;
	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//注意！不能使用if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)来判断
    {
        USART_ReceiveData(USART1);
    }
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		res =USART_ReceiveData(USART1);
                R1V2_RX_BUF[Rx_Sta] = res;
                Rx_Sta++;
               if(R1V2_RX_BUF[0] != 0x0F)
               {
                 for (u8 i = 0; i<25; i++)
                 {		//清空缓存区
                       R1V2_RX_BUF[i] = 0;  
                       break;
                  }
                  Rx_Sta=0;
                }
          if (R1V2_RX_BUF[0] == 0x0F && R1V2_RX_BUF[24] == 0x00 && Rx_Sta == 25)
          {
            R1V2_Data_Count(R1V2_RX_BUF);
            for (u8 i = 0; i<25; i++)		//清空缓存区
              R1V2_RX_BUF[i] = 0;  
            Rx_Sta=0;
          }
	}
        USART_ClearITPendingBit(USART1,USART_IT_RXNE); // 清中断标志
        OSIntExit(); // Tell uC/OS-III that we are leaving the ISR 

}  

///*
//******************************************************************************
//*    函 数 名    ：R1V2_Data_Count(uint8_t *buf)
//*    描    述    ：分析R1V2数据
//*    输入参数    ：
//*           buf:R1V2数据缓存区
//******************************************************************************
//*/


void R1V2_Data_Count(uint8_t *buf)
{        
        
        CH[0] = ((int16_t)buf[ 1] >> 0 | ((int16_t)buf[ 2] << 8 )) & 0x07FF;
        CH[1] = ((int16_t)buf[ 2] >> 3 | ((int16_t)buf[ 3] << 5 )) & 0x07FF;
        CH[2] = ((int16_t)buf[ 3] >> 6 | ((int16_t)buf[ 4] << 2 ) | (int16_t)buf[ 5] << 10 ) & 0x07FF;
        CH[3] = ((int16_t)buf[ 5] >> 1 | ((int16_t)buf[ 6] << 7 )) & 0x07FF;
        CH[4] = ((int16_t)buf[ 6] >> 4 | ((int16_t)buf[ 7] << 4 )) & 0x07FF;
        CH[5] = ((int16_t)buf[ 7] >> 7 | ((int16_t)buf[ 8] << 1 ) | (int16_t)buf[9] << 9 ) & 0x07FF;
        CH[6] = ((int16_t)buf[ 9] >> 2 | ((int16_t)buf[10] << 6 )) & 0x07FF;
        CH[7] = ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3 )) & 0x07FF;
        CH[8] = ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8 )) & 0x07FF;
        CH[9] = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5 )) & 0x07FF;
        CH[10] = ((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2 ) | (int16_t)buf[16] << 10 ) & 0x07FF;
        CH[11] = ((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7 )) & 0x07FF;
        CH[12] = ((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4 )) & 0x07FF;
        CH[13] = ((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1 ) | (int16_t)buf[20] << 9 ) & 0x07FF;
        CH[14] = ((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6 )) & 0x07FF;
        CH[15] = ((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3 )) & 0x07FF;
}