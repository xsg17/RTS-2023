
//*******************************************************************************************************
//                                            INCLUDE FILES
//*******************************************************************************************************


#define  CAN_MODULE
#include "can.h"


//*******************************************************************************************************
//                                           	DEFINES
//*******************************************************************************************************
//
//
//
//*******************************************************************************************************
//Brief    : 初始化 CAN 硬件
//
//Param(s) : none.
//
//Return(s): none.
//
//*******************************************************************************************************


void CAN1_Init (void) //初始化CAN 1
{
  GPIO_InitTypeDef GPIO_InitStructure;
  CAN_InitTypeDef CAN_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  
  CAN1_PORT_ENABLE();
  
  GPIO_InitStructure.GPIO_Pin = PIN_CAN1_TX | PIN_CAN1_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( PORT_CAN1, &GPIO_InitStructure );
  
  GPIO_PinAFConfig( PORT_CAN1, CAN1_AF_TX, CAN1_AF );
  GPIO_PinAFConfig( PORT_CAN1, CAN1_AF_RX, CAN1_AF );
  
  CAN_StructInit( &CAN_InitStructure );
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = ENABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
  CAN_InitStructure.CAN_Prescaler = 5; // Braudrate = 42M/((1+9+4)*3) = 1M
  CAN_Init( CAN1, &CAN_InitStructure );
  
  
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit( &CAN_FilterInitStructure );
  
  
  
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );
  
  
  CAN_ITConfig( CAN1, CAN_IT_FMP0 | CAN_IT_FOV0, ENABLE );
}

void CAN2_Init (void) //初始化CAN 2
{
  GPIO_InitTypeDef GPIO_InitStructure;
  CAN_InitTypeDef CAN_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  
  CAN2_PORT_ENABLE();
  
  GPIO_InitStructure.GPIO_Pin = PIN_CAN2_TX | PIN_CAN2_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init( PORT_CAN2, &GPIO_InitStructure );
  
  GPIO_PinAFConfig( PORT_CAN2, CAN2_AF_TX, CAN2_AF );
  GPIO_PinAFConfig( PORT_CAN2, CAN2_AF_RX, CAN2_AF );
  
  CAN_StructInit(&CAN_InitStructure);
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = ENABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
  CAN_InitStructure.CAN_Prescaler = 3; // Braudrate = 42M/((1+9+4)*3) = 1M
  CAN_Init( CAN2, &CAN_InitStructure );
  
  // Configure CAN filter 
  CAN_FilterInitStructure.CAN_FilterNumber = 14;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit( &CAN_FilterInitStructure );
  
  	CAN_FilterInitStructure.CAN_FilterNumber = 15;
  	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
  	CAN_FilterInitStructure.CAN_FilterIdLow = 0;
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0;
  	CAN_FilterInitStructure.CAN_FilterIdHigh = 0;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0;
  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO1;
  	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  	CAN_FilterInit( &CAN_FilterInitStructure );
  
//   Enable CAN2 RX0 interrupt IRQ channel 
  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );
  
  	// Enable CAN2 RX1 interrupt IRQ channel 
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
  
  CAN_ITConfig(CAN2, CAN_IT_FMP1 | CAN_IT_FOV1, ENABLE);  // overrun, message pending intterupt
}

/*
*********************************************************************************************************
* Brief    : Send CAN message.
*
* Param(s) : CAN_ID - 11 bits of can standard ID
*			 msg    - pointer to an 8 bytes message
*
* Return(s): none.
*
*********************************************************************************************************
*/

uint8_t CAN1_Send_Msg ( uint16_t CAN_ID, uint8_t *msg )
{
  uint8_t i;
   CanTxMsg TxMessage;


  TxMessage.StdId = CAN_ID;
  TxMessage.ExtId = 0x00000000;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.DLC = 8;
  for( i = 0; i < 8; i++ )
  {
    TxMessage.Data[i] = msg[i];
  }
  return CAN_Transmit(CAN1, &TxMessage);
}

uint8_t CAN1_Send_Msg_ExtendID  ( uint16_t CAN_ID, uint8_t *msg )
{
  uint8_t i;
  CanTxMsg TxMessage;
  TxMessage.StdId = 0x00000000;
  TxMessage.ExtId = CAN_ID;
  TxMessage.IDE = CAN_ID_EXT;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.DLC = 8;
  for( i = 0; i < 8; i++ )
  {
    TxMessage.Data[i] = msg[i];
  }
  return CAN_Transmit(CAN1, &TxMessage);
}




uint8_t CAN2_Send_Msg (uint16_t CAN_ID, uint8_t *msg)
{
  uint8_t i;
  CanTxMsg TxMessage;
  
  TxMessage.StdId = CAN_ID;
  TxMessage.ExtId = 0x00000000;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.DLC = 8;
  for(i = 0; i < 8; i++)
  {
    TxMessage.Data[i] = msg[i];
  }
}


/*
*********************************************************************************************************
* Brief    : 此函数处理can消息挂起处理程序
*
* Param(s) : none
*
* Return(s): none.
*
*********************************************************************************************************
*/
CanRxMsg testRxMessage;
void CAN1_RX0_IRQHandler (void)
{
  static uint8_t overrun = 0;
  CanRxMsg RxMessage;
  
  CPU_SR_ALLOC();
  
  CPU_CRITICAL_ENTER();
  OSIntEnter(); // Tell uC/OS-III that we are starting an ISR   
  CPU_CRITICAL_EXIT();
  
  if(CAN_GetITStatus( CAN1, CAN_IT_FMP0 ) != RESET )
  {   
    CAN_Receive( CAN1, CAN_FIFO0, &RxMessage );
    testRxMessage=RxMessage;
    

  if (RxMessage.IDE == CAN_ID_STD ||RxMessage.StdId==1||RxMessage.StdId==2||RxMessage.StdId==3)
   {      Motor_Analyze(&RxMessage);
  }

   else if (RxMessage.IDE == CAN_ID_EXT)
    {
     Motor_Analyze_vesc(&RxMessage);
    }
       
    
    
    CAN_ClearITPendingBit( CAN1, CAN_IT_FMP0 );
  }
  
  if( CAN_GetITStatus( CAN1, CAN_IT_FOV0 ) != RESET )
  {
    overrun++;
    CAN_ClearITPendingBit( CAN1, CAN_IT_FOV0 );
  }
  
  OSIntExit(); // Tell uC/OS-III that we are leaving the ISR 
}
void CAN1_RX1_IRQHandler (void)
{
  static uint8_t overrun = 0;
  CanRxMsg RxMessage;
  
  CPU_SR_ALLOC();
  
  CPU_CRITICAL_ENTER();
  OSIntEnter(); // Tell uC/OS-III that we are starting an ISR    
  CPU_CRITICAL_EXIT();
  
  if( CAN_GetITStatus( CAN1, CAN_IT_FMP1 ) != RESET )
  {
    
     CAN_Receive( CAN1, CAN_FIFO0, &RxMessage );
    
     testRxMessage=RxMessage;
    if (RxMessage.IDE == CAN_ID_STD || RxMessage.StdId==1||RxMessage.StdId==2||RxMessage.StdId==3)
    {
      Motor_Analyze(&RxMessage);
    }

    else if (RxMessage.IDE == CAN_ID_EXT)
    {
      Motor_Analyze_vesc(&RxMessage);
    }

    CAN_ClearITPendingBit( CAN1, CAN_IT_FMP1 );
  }
  
  if(CAN_GetITStatus( CAN1, CAN_IT_FOV1 ) != RESET )
  {
    overrun++;
    CAN_ClearITPendingBit( CAN1, CAN_IT_FOV1 );
  }
  
  OSIntExit(); // Tell uC/OS-III that we are leaving the ISR 
}


void CAN2_RX0_IRQHandler (void)
{
  static uint8_t overrun = 0;
  CanRxMsg RxMessage;
  
  CPU_SR_ALLOC();
  
  CPU_CRITICAL_ENTER();
  OSIntEnter(); // Tell uC/OS-III that we are starting an ISR   
  CPU_CRITICAL_EXIT();
  
  if(CAN_GetITStatus( CAN2, CAN_IT_FMP0 ) != RESET )
  {
    CAN_Receive( CAN2, CAN_FIFO0, &RxMessage );
    testRxMessage=RxMessage;

    Motor_Analyze_C610(&RxMessage);
    
    CAN_ClearITPendingBit( CAN2, CAN_IT_FMP0 );
  }
  
  if( CAN_GetITStatus( CAN2, CAN_IT_FOV0 ) != RESET )
  {
    overrun++;
    CAN_ClearITPendingBit( CAN2, CAN_IT_FOV0 );
  }
  
  OSIntExit(); // Tell uC/OS-III that we are leaving the ISR 
}
void CAN2_RX1_IRQHandler (void)
{
  static uint8_t overrun = 0;
  CanRxMsg RxMessage;
  
  CPU_SR_ALLOC();
  
  CPU_CRITICAL_ENTER();
  OSIntEnter(); // Tell uC/OS-III that we are starting an ISR    
  CPU_CRITICAL_EXIT();
  
  if( CAN_GetITStatus( CAN2, CAN_IT_FMP1 ) != RESET )
  {
    CAN_Receive( CAN2, CAN_FIFO1, &RxMessage );
    testRxMessage=RxMessage;

    Motor_Analyze_C610(&RxMessage);

    CAN_ClearITPendingBit( CAN2, CAN_IT_FMP1 );
  }
  
  if(CAN_GetITStatus( CAN2, CAN_IT_FOV1 ) != RESET )
  {
    overrun++;
    CAN_ClearITPendingBit( CAN2, CAN_IT_FOV1 );
  }
  
  OSIntExit(); // Tell uC/OS-III that we are leaving the ISR 
}





//
//
//void CAN1_Init (void) //初始化CAN 1
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  CAN_InitTypeDef CAN_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
//  CAN_FilterInitTypeDef CAN_FilterInitStructure;
//  
//  CAN1_PORT_ENABLE();
//  
//  GPIO_InitStructure.GPIO_Pin = PIN_CAN1_TX | PIN_CAN1_RX;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init( PORT_CAN1, &GPIO_InitStructure );
//  
//  GPIO_PinAFConfig( PORT_CAN1, CAN1_AF_TX, CAN1_AF );
//  GPIO_PinAFConfig( PORT_CAN1, CAN1_AF_RX, CAN1_AF );
//  
//  CAN_StructInit( &CAN_InitStructure );
//  CAN_InitStructure.CAN_TTCM = DISABLE;
//  CAN_InitStructure.CAN_ABOM = DISABLE;
//  CAN_InitStructure.CAN_AWUM = DISABLE;
//  CAN_InitStructure.CAN_NART = ENABLE;
//  CAN_InitStructure.CAN_RFLM = DISABLE;
//  CAN_InitStructure.CAN_TXFP = DISABLE;
//  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
//  
//  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
//  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
//  CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
//  CAN_InitStructure.CAN_Prescaler = 5; // Braudrate = 45M/((1+3+5)*5) = 1M
//  CAN_Init( CAN1, &CAN_InitStructure );
//  
//  CAN_FilterInitStructure.CAN_FilterNumber = 0;
//  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
//  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
//  CAN_FilterInitStructure.CAN_FilterIdLow = 0;
//  CAN_FilterInitStructure.CAN_FilterIdHigh = 0;
//  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0;
//  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0;
//  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
//  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
//  CAN_FilterInit( &CAN_FilterInitStructure );
//  
//  
//  
//  // Enable CAN1 RX1 interrupt IRQ channel 
//  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init( &NVIC_InitStructure );
//  
//  CAN_ITConfig( CAN1, CAN_IT_FMP0 | CAN_IT_FOV0, ENABLE );
//}
//
//void CAN2_Init (void) //初始化CAN 2
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  CAN_InitTypeDef CAN_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
//  CAN_FilterInitTypeDef CAN_FilterInitStructure;
//  
//  CAN2_PORT_ENABLE();
//  
//  GPIO_InitStructure.GPIO_Pin = PIN_CAN2_TX | PIN_CAN2_RX;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_Init( PORT_CAN2, &GPIO_InitStructure );
//  
//  GPIO_PinAFConfig( PORT_CAN2, CAN2_AF_TX, GPIO_AF_CAN2 );
//  GPIO_PinAFConfig( PORT_CAN2, CAN2_AF_RX, GPIO_AF_CAN2 );
//  
//  CAN_StructInit(&CAN_InitStructure);
//  CAN_InitStructure.CAN_TTCM = DISABLE;
//  CAN_InitStructure.CAN_ABOM = DISABLE;
//  CAN_InitStructure.CAN_AWUM = DISABLE;
//  CAN_InitStructure.CAN_NART = DISABLE;
//  CAN_InitStructure.CAN_RFLM = DISABLE;
//  CAN_InitStructure.CAN_TXFP = ENABLE;
//  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
//  
//  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
//  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
//  CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
//  CAN_InitStructure.CAN_Prescaler = 5; // Braudrate = 45M/((1+3+5)*5) = 1M
//  CAN_Init( CAN2, &CAN_InitStructure );
//  
//  // Configure CAN filter 
//  CAN_FilterInitStructure.CAN_FilterNumber = 15;
//  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
//  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
//  CAN_FilterInitStructure.CAN_FilterIdLow = 0;
//  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0;
//  CAN_FilterInitStructure.CAN_FilterIdHigh = 0;
//  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0;
//  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO1;
//  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
//  CAN_FilterInit( &CAN_FilterInitStructure );
//  
////   Enable CAN2 RX0 interrupt IRQ channel 
//  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init( &NVIC_InitStructure );
//  
//  CAN_ITConfig(CAN2, CAN_IT_FMP1 | CAN_IT_FOV1, ENABLE);  // overrun, message pending intterupt
//}
//
///*
//*********************************************************************************************************
//* Brief    : Send CAN message.
//*
//* Param(s) : CAN_ID - 11 bits of can standard ID
//*			 msg    - pointer to an 8 bytes message
//*
//* Return(s): none.
//*
//*********************************************************************************************************
//*/
//
//uint8_t CAN1_Send_Msg ( uint16_t CAN_ID, uint8_t *msg )
//{
//  uint8_t i;
//  CanTxMsg TxMessage;
//  
//
//  TxMessage.StdId = CAN_ID;
//  TxMessage.ExtId = 0x00000000;
//  TxMessage.IDE = CAN_ID_STD;
//  TxMessage.RTR = CAN_RTR_DATA;
//  TxMessage.DLC = 8;
//  for( i = 0; i < 8; i++ )
//  {
//    TxMessage.Data[i] = msg[i];
//  }
//  return   CAN_Transmit(CAN1, &TxMessage);
//   
//}
//
//uint8_t CAN2_Send_Msg (uint16_t CAN_ID, uint8_t *msg)
//{
//  
//  uint8_t i;
//  CanTxMsg TxMessage;
//   
//  TxMessage.StdId = CAN_ID;
//  TxMessage.ExtId = 0x00000000;
//  TxMessage.IDE = CAN_ID_STD;
//  TxMessage.RTR = CAN_RTR_DATA;
//  TxMessage.DLC = 8;
//  for(i = 0; i < 8; i++)
//  {
//    TxMessage.Data[i] = msg[i];
//  }
//  return CAN_Transmit(CAN2, &TxMessage);
//
//}
//
//
///*
//*********************************************************************************************************
//* Brief    : 此函数处理can消息挂起处理程序
//*
//* Param(s) : none
//*
//* Return(s): none.
//*
//*********************************************************************************************************
//*/
//CanRxMsg testRxMessage;
//void CAN1_RX0_IRQHandler (void)
//{
//  static uint8_t overrun = 0;
//  CanRxMsg RxMessage;
//  
//  CPU_SR_ALLOC();
//
//  CPU_CRITICAL_ENTER();
//  OSIntEnter(); // Tell uC/OS-III that we are starting an ISR   
//  CPU_CRITICAL_EXIT();
//  
//  if(CAN_GetITStatus( CAN1, CAN_IT_FMP0 ) != RESET )
//  {
//    CAN_Receive( CAN1, CAN_FIFO0, &RxMessage );
//    testRxMessage=RxMessage;
//    //VEGA_CANAnalyze(&RxMessage);
//    if(RxMessage.StdId==1||RxMessage.StdId==2||RxMessage.StdId==3)
//    {
//     // Encoder_Analyze(&RxMessage);
//    }
//    else
//    {
//      Motor_Analyze(&RxMessage);
//    }
//  
//    CAN_ClearITPendingBit( CAN1, CAN_IT_FMP0 );
//  }
//  
//  if( CAN_GetITStatus( CAN1, CAN_IT_FOV0 ) != RESET )
//  {
//    overrun++;
//    CAN_ClearITPendingBit( CAN1, CAN_IT_FOV0 );
//  }
//  
//  OSIntExit(); // Tell uC/OS-III that we are leaving the ISR 
//}
//void CAN1_RX1_IRQHandler (void)
//{
//  static uint8_t overrun = 0;
//  CanRxMsg RxMessage;
//  
//  CPU_SR_ALLOC();
//  
//  CPU_CRITICAL_ENTER();
//  OSIntEnter(); // Tell uC/OS-III that we are starting an ISR    
//  CPU_CRITICAL_EXIT();
//  
//  if( CAN_GetITStatus( CAN1, CAN_IT_FMP1 ) != RESET )
//  {
//    CAN_Receive( CAN1, CAN_FIFO1, &RxMessage );
//    testRxMessage=RxMessage;
//    if(RxMessage.StdId==1||RxMessage.StdId==2||RxMessage.StdId==3)
//    {
//     // Encoder_Analyze(&RxMessage);
//    }
//    else
//    {
//      Motor_Analyze(&RxMessage);
//    }
//  
//    CAN_ClearITPendingBit( CAN1, CAN_IT_FMP1 );
//  }
//  
//  if(CAN_GetITStatus( CAN1, CAN_IT_FOV1 ) != RESET )
//  {
//    overrun++;
//    CAN_ClearITPendingBit( CAN1, CAN_IT_FOV1 );
//  }
//  
//  OSIntExit(); // Tell uC/OS-III that we are leaving the ISR 
//}
//
//void CAN2_RX0_IRQHandler (void)
//{
//  static uint8_t overrun = 0;
//  CanRxMsg RxMessage;
//  
//  CPU_SR_ALLOC();
//  
//  CPU_CRITICAL_ENTER();
//  OSIntEnter(); // Tell uC/OS-III that we are starting an ISR   
//  CPU_CRITICAL_EXIT();
//  
//  if(CAN_GetITStatus( CAN2, CAN_IT_FMP0 ) != RESET )
//  {
//    CAN_Receive( CAN2, CAN_FIFO0, &RxMessage );
//    testRxMessage=RxMessage;
//
//    Motor_Analyze_C610(&RxMessage);
//    
//    CAN_ClearITPendingBit( CAN2, CAN_IT_FMP0 );
//  }
//  
//  if( CAN_GetITStatus( CAN2, CAN_IT_FOV0 ) != RESET )
//  {
//    overrun++;
//    CAN_ClearITPendingBit( CAN2, CAN_IT_FOV0 );
//  }
//  
//  OSIntExit(); // Tell uC/OS-III that we are leaving the ISR 
//}
//void CAN2_RX1_IRQHandler (void)
//{
//  static uint8_t overrun = 0;
//  CanRxMsg RxMessage;
//  
//  CPU_SR_ALLOC();
//  
//  CPU_CRITICAL_ENTER();
//  OSIntEnter(); // Tell uC/OS-III that we are starting an ISR    
//  CPU_CRITICAL_EXIT();
//  
//  if( CAN_GetITStatus( CAN2, CAN_IT_FMP1 ) != RESET )
//  {
//    CAN_Receive( CAN2, CAN_FIFO1, &RxMessage );
//    testRxMessage=RxMessage;
//
//    Motor_Analyze_C610(&RxMessage);
//
//    CAN_ClearITPendingBit( CAN2, CAN_IT_FMP1 );
//  }
//  
//  if(CAN_GetITStatus( CAN2, CAN_IT_FOV1 ) != RESET )
//  {
//    overrun++;
//    CAN_ClearITPendingBit( CAN2, CAN_IT_FOV1 );
//  }
//  
//  OSIntExit(); // Tell uC/OS-III that we are leaving the ISR 
//}
//
