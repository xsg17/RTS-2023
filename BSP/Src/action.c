/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  ACTION_MODULE
#include "action.h"

/*
*********************************************************************************************************
*                                           	 DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
* Brief    : Initialize  USART to receive data from the module.
*
* Param(s) : none.
*
* Return(s): none.
*
*********************************************************************************************************
*/

void Action_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  USART_InitTypeDef USART_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  
  // Peripheral Clock Enable
  ACTION_PORT_ENABLE();
  
  // USART GPIO configuration 
  GPIO_PinAFConfig( PORT_ACTION, ACTION_AF_TX, ACTION_AF_USART );
  GPIO_PinAFConfig( PORT_ACTION, ACTION_AF_RX, ACTION_AF_USART );
  
  GPIO_InitStruct.GPIO_Pin = ACTION_PIN_TX | ACTION_PIN_RX;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init( PORT_ACTION, &GPIO_InitStruct );
  
  // Enable the USART OverSampling by 8 
  USART_OverSampling8Cmd( ACTION_USART, ENABLE );
  
  // USART configuration 
  USART_InitStruct.USART_BaudRate = ACTION_BRAUDRATE;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init( ACTION_USART, &USART_InitStruct );
  
  NVIC_InitStruct.NVIC_IRQChannel = USART_ACTION_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStruct );
  
  // Enable USART 
  USART_Cmd(ACTION_USART, ENABLE);
  USART_ITConfig( ACTION_USART, USART_IT_RXNE, ENABLE );
}

/*
*********************************************************************************************************
* Brief    : This function handles USART interrupt (receive & transmit).
*
* Param(s) : none
*
* Return(s): none.
*
*********************************************************************************************************
*/

void USART_ACTION_IRQHandler(void)
{
  static uint8_t ch;
  static uint8_t step = 0, count = 0;
  static union
  {
    uint8_t data[24];
    float ActVal[6];
    
  }posture;
  
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  OSIntEnter(); // Tell uC/OS-III that we are starting an ISR
  CPU_CRITICAL_EXIT();
  
  // USART in Receiver mode 
  if( USART_GetITStatus( ACTION_USART, USART_IT_RXNE ) == SET )
  {
    USART_ClearITPendingBit( ACTION_USART, USART_IT_RXNE );
    ch = USART_ReceiveData( ACTION_USART );
    switch(step)
    {
    case 0:
      if(ch == 0x0d) { step++; }
      else
        step=0;
      break;
      
    case 1:
      if(ch == 0x0a)
      {
        count = 0;
        step++;
      }
      else if(ch == 0x0d);
      else
        step = 0;
      break;
      
    case 2:
      posture.data[count] = ch;
      count++;
      if( count >= 24 )
      {
        count = 0;
        step++;
      }
      break;
      
    case 3:
      if( ch == 0x0a ) { step++; }
      else
        step = 0;
      break;
      
    case 4:
      if( ch == 0x0d )
      {
        RobotPos.zangle = posture.ActVal[0];
        RobotPos.xangle = posture.ActVal[1];
        RobotPos.yangle = posture.ActVal[2];
        RobotPos.pos_x  = posture.ActVal[3];
        RobotPos.pos_y  = posture.ActVal[4];
        RobotPos.w_z    = posture.ActVal[5];
        
        RobotPos.theta = PI*RobotPos.zangle / 180.0;  //从底层模块读取到的角度信息
        RobotPos.theta = ( (int)( 100000.0 * RobotPos.theta ) ) / 100000.0;//精度截取
        
       
        if( flag_arbitraryPos == 1)
        {
           Robot_Pos_Ctrl();
           
        }
        Robot_Speed_Update();
    //    Robot_Speed_Ctrl(); // 速度更新
        
        SyncSignal();//发送同步信号
      }
      step = 0;
      break;
      
    default:
      step = 0;
      break;
    }
    FeedbackData = 0;
  }
  OSIntExit(); // Tell uC/OS-III that we are leaving the ISR
}

/*
*********************************************************************************************************
*    函    数   ：   Action_Send_Char(u8 Char)
*    描    述   ：   向Action模块发送一个字符
*********************************************************************************************************
*/
void Action_Send_Char(u8 Char)
{
  USART_SendData( ACTION_USART, Char );
  // 等待发送完成一个字节
  while( USART_GetFlagStatus( ACTION_USART, USART_FLAG_TXE ) != SET );
}

/*
*********************************************************************************************************
*                     Action_Reinit()
*    Brief    : 将action模块所有坐标角度设为0
*********************************************************************************************************
*/
void Action_Reinit(void)
{
  Action_Send_Char('A');
  Action_Send_Char('C');
  Action_Send_Char('T');
  Action_Send_Char('0');
}

/*
*********************************************************************************************************
Action_AngleReinit()
* Brief    : 将action模块所有坐标角度设为0
*********************************************************************************************************
*/
void Action_AngleReinit(void)
{
  Action_Send_Char('A');
  Action_Send_Char('C');
  Action_Send_Char('T');
  Action_Send_Char('J');
  Action_Send_Char(0x00);
  Action_Send_Char(0x00);
  Action_Send_Char(0x00);
  Action_Send_Char(0x00);
}