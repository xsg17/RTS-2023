/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  USART_MODULE
#include "usart.h"

unsigned char  ReceiveData,ReceiveData_2;

/*
************************************************************************************************************
*                                            usart_init()
************************************************************************************************************
*/
      
//void usart_init(void) // 初始化 USART3 （ PD8，PD9 ）串口通信
//{
//        GPIO_InitTypeDef GPIO_InitStructure;
//        USART_InitTypeDef USART_InitStructure;
//        NVIC_InitTypeDef NVIC_InitStructure;
//        //GPIOD 和 USART3 时钟使能①
//        
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); // 使能 GPIOA 时钟
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); // 使能 USART3 时钟
//        // USART_DeInit(USART3); // 复位串口 1 ②
//        
//        GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); // PD9 复用为 USART3
//        GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); // PD8 复用为 USART3
//        // USART3_TX PD.8 PD.9 ③
//        
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8; // GPIOD9 与 GPIOD8
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 复用功能
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速度 50MHz
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽复用输出
//        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
//        GPIO_Init(GPIOD,&GPIO_InitStructure); // 初始化 PD9，PD8
//        
//        USART_InitStructure.USART_BaudRate = bound; // 一般设置为 115200;
//        USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长为 8 位数据格式
//        USART_InitStructure.USART_StopBits = USART_StopBits_1; // 一个停止位
//        USART_InitStructure.USART_Parity = USART_Parity_No; // 无奇偶校验位
//        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
//        USART_Init(USART3, &USART_InitStructure); // 初始化串口
//
//        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 开启中断
//        // USART3 NVIC 配置
//        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
//        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级 2
//        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // 响应优先级 2
//
//        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ 通道使能
//        NVIC_Init(&NVIC_InitStructure);  // 根据指定的参数初始化 VIC 寄存器
//        
//        USART_Cmd(USART3, ENABLE); // 使能串口
//}
//
//void send_uchar(unsigned char udata) // USART3 uchar 型数据发送
//{
//  USART_SendData(USART3, udata);
//  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);
//}
//
//void send_char(char data) // USART3 char 型数据发送
//{
//  USART_SendData(USART3, data);
//  while( USART_GetFlagStatus( USART3, USART_FLAG_TC ) != SET );
//}
//
//void SendToDraw( uint8_t data8_H1, uint8_t data8_L1, //数据1 
//                uint8_t data8_H2, uint8_t data8_L2,  //数据2
//                uint8_t data8_H3, uint8_t data8_L3,  //数据3
//                uint8_t data8_H4, uint8_t data8_L4 )  //数据4 
//{ // 传递给上位机模拟示波器画图
//  //数据头
//  send_uchar( 0x03 );
//  send_uchar( 0xfc );
//  //数据1
//  send_uchar( data8_L1 );
//  send_uchar( data8_H1 );
//  //数据2
//  send_uchar( data8_L2 );
//  send_uchar( data8_H2 ); 
//  //数据3
//  send_uchar( data8_L3 );
//  send_uchar( data8_H3 );
//  //数据4
//  send_uchar( data8_L4 );
//  send_uchar( data8_H4 );
//  //数据尾
//  send_uchar( 0xfc );
//  send_uchar( 0x03 );
//}

//void usart_init_draw(void) // 初始化 USART6 （ PC6，PC7 ）串口通信
//{
//  GPIO_InitTypeDef GPIO_InitStructure;  // 声明一个结构体变量，用来初始化GPIO 
//  USART_InitTypeDef USART_InitStructure;  // 中断结构体定义  
//  NVIC_InitTypeDef NVIC_InitStructure; // 串口结构体定义    
//  
//  // GPIOC 和 USART6 时钟使能
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能 GPIOC时钟
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE); //使能 USART6 时钟
//
//  
//  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); // PC6 复用为 USART6
//  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); // PC7 复用为 USART6
//
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ; // GPIOC6 与 GPIOC7
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 复用功能
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速度 50MHz
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽复用输出
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
//  GPIO_Init(GPIOC,&GPIO_InitStructure); // 初始化 PC6，PC7
//  
//  USART_InitStructure.USART_BaudRate = bound; // 一般设置为 115200;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长为 8 位数据格式
//  USART_InitStructure.USART_StopBits = USART_StopBits_1; // 一个停止位
//  USART_InitStructure.USART_Parity = USART_Parity_No; // 无奇偶校验位
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // 收发模式
//  USART_Init(USART6, &USART_InitStructure); // 初始化串口
//
//  // USART6 NVIC 配置
//  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级 2
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // 响应优先级 2
//  
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ 通道使能
//  NVIC_Init(&NVIC_InitStructure);  // 根据指定的参数初始化 VIC 寄存器
//  
//  USART_Cmd(USART6, ENABLE); // 使能串口
//  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // 开启中断
//}

//void send_uchar_draw(unsigned char data2) // USART6 uchar 型数据发送
//{
//  USART_SendData(USART6, data2);
//  while( USART_GetFlagStatus( USART6, USART_FLAG_TC ) != SET );
//}
//
//void SendToDraw2( uint8_t data8_H1, uint8_t data8_L1, uint8_t data8_H2, uint8_t data8_L2 , uint8_t data8_H3,uint8_t data8_L3,uint8_t data8_H4,uint8_t data8_L4) // 传递给上位机画图
//{
//  //数据头
//  send_uchar( 0x03 );
//  send_uchar( 0xfc );
//  //数据1
//  send_uchar( data8_H1 ); 
//  send_uchar( data8_L1 );
//   //数据2
//  send_uchar( data8_H2 ); 
//  send_uchar( data8_L2 );
// 
//   send_uchar( data8_H3 ); 
//  send_uchar( data8_L3 );
//  
//   send_uchar( data8_H4 ); 
//  send_uchar( data8_L4);     
//   //数据尾
//  send_uchar( 0xfc );
//  send_uchar( 0x03 );
//}

//void USART3_IRQHandler(void) // USART3 中断
//{
//  // 接收模式下的USART3
//  if( USART_GetITStatus( USART3, USART_IT_RXNE ) != RESET ) 
//  {
//    ReceiveData = USART_ReceiveData(USART3); // 接收数据
////    USART_SendData( USART3, ReceiveData ); // 发送接收到的数据
//  }
//  
//  USART_ClearITPendingBit( USART3, USART_IT_RXNE );
//}

//void USART6_IRQHandler(void) // USART6 中断
//{
//  //接收模式下的USART6
//  if( USART_GetITStatus( USART6, USART_IT_RXNE ) != RESET ) 
//  { 
//    USART_ClearITPendingBit( USART6, USART_IT_RXNE );
//    ReceiveData_2 = USART_ReceiveData( USART6 ); // 接收数据
//
//    WirelessParaAdj( ReceiveData_2 ); // 无线调参 PID 接收数据
//    
//    USART_SendData( USART6, ReceiveData_2 ); //发送接收到的数据
//  }
//  USART_ClearITPendingBit( USART6, USART_IT_RXNE );
//}
