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
      
//void usart_init(void) // ��ʼ�� USART3 �� PD8��PD9 ������ͨ��
//{
//        GPIO_InitTypeDef GPIO_InitStructure;
//        USART_InitTypeDef USART_InitStructure;
//        NVIC_InitTypeDef NVIC_InitStructure;
//        //GPIOD �� USART3 ʱ��ʹ�ܢ�
//        
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); // ʹ�� GPIOA ʱ��
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); // ʹ�� USART3 ʱ��
//        // USART_DeInit(USART3); // ��λ���� 1 ��
//        
//        GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); // PD9 ����Ϊ USART3
//        GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); // PD8 ����Ϊ USART3
//        // USART3_TX PD.8 PD.9 ��
//        
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8; // GPIOD9 �� GPIOD8
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // ���ù���
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // �ٶ� 50MHz
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // ���츴�����
//        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // ����
//        GPIO_Init(GPIOD,&GPIO_InitStructure); // ��ʼ�� PD9��PD8
//        
//        USART_InitStructure.USART_BaudRate = bound; // һ������Ϊ 115200;
//        USART_InitStructure.USART_WordLength = USART_WordLength_8b; // �ֳ�Ϊ 8 λ���ݸ�ʽ
//        USART_InitStructure.USART_StopBits = USART_StopBits_1; // һ��ֹͣλ
//        USART_InitStructure.USART_Parity = USART_Parity_No; // ����żУ��λ
//        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // �շ�ģʽ
//        USART_Init(USART3, &USART_InitStructure); // ��ʼ������
//
//        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // �����ж�
//        // USART3 NVIC ����
//        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
//        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // ��ռ���ȼ� 2
//        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // ��Ӧ���ȼ� 2
//
//        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ ͨ��ʹ��
//        NVIC_Init(&NVIC_InitStructure);  // ����ָ���Ĳ�����ʼ�� VIC �Ĵ���
//        
//        USART_Cmd(USART3, ENABLE); // ʹ�ܴ���
//}
//
//void send_uchar(unsigned char udata) // USART3 uchar �����ݷ���
//{
//  USART_SendData(USART3, udata);
//  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);
//}
//
//void send_char(char data) // USART3 char �����ݷ���
//{
//  USART_SendData(USART3, data);
//  while( USART_GetFlagStatus( USART3, USART_FLAG_TC ) != SET );
//}
//
//void SendToDraw( uint8_t data8_H1, uint8_t data8_L1, //����1 
//                uint8_t data8_H2, uint8_t data8_L2,  //����2
//                uint8_t data8_H3, uint8_t data8_L3,  //����3
//                uint8_t data8_H4, uint8_t data8_L4 )  //����4 
//{ // ���ݸ���λ��ģ��ʾ������ͼ
//  //����ͷ
//  send_uchar( 0x03 );
//  send_uchar( 0xfc );
//  //����1
//  send_uchar( data8_L1 );
//  send_uchar( data8_H1 );
//  //����2
//  send_uchar( data8_L2 );
//  send_uchar( data8_H2 ); 
//  //����3
//  send_uchar( data8_L3 );
//  send_uchar( data8_H3 );
//  //����4
//  send_uchar( data8_L4 );
//  send_uchar( data8_H4 );
//  //����β
//  send_uchar( 0xfc );
//  send_uchar( 0x03 );
//}

//void usart_init_draw(void) // ��ʼ�� USART6 �� PC6��PC7 ������ͨ��
//{
//  GPIO_InitTypeDef GPIO_InitStructure;  // ����һ���ṹ�������������ʼ��GPIO 
//  USART_InitTypeDef USART_InitStructure;  // �жϽṹ�嶨��  
//  NVIC_InitTypeDef NVIC_InitStructure; // ���ڽṹ�嶨��    
//  
//  // GPIOC �� USART6 ʱ��ʹ��
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ�� GPIOCʱ��
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE); //ʹ�� USART6 ʱ��
//
//  
//  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); // PC6 ����Ϊ USART6
//  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); // PC7 ����Ϊ USART6
//
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ; // GPIOC6 �� GPIOC7
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // ���ù���
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // �ٶ� 50MHz
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // ���츴�����
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // ����
//  GPIO_Init(GPIOC,&GPIO_InitStructure); // ��ʼ�� PC6��PC7
//  
//  USART_InitStructure.USART_BaudRate = bound; // һ������Ϊ 115200;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b; // �ֳ�Ϊ 8 λ���ݸ�ʽ
//  USART_InitStructure.USART_StopBits = USART_StopBits_1; // һ��ֹͣλ
//  USART_InitStructure.USART_Parity = USART_Parity_No; // ����żУ��λ
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // �շ�ģʽ
//  USART_Init(USART6, &USART_InitStructure); // ��ʼ������
//
//  // USART6 NVIC ����
//  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // ��ռ���ȼ� 2
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // ��Ӧ���ȼ� 2
//  
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ ͨ��ʹ��
//  NVIC_Init(&NVIC_InitStructure);  // ����ָ���Ĳ�����ʼ�� VIC �Ĵ���
//  
//  USART_Cmd(USART6, ENABLE); // ʹ�ܴ���
//  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // �����ж�
//}

//void send_uchar_draw(unsigned char data2) // USART6 uchar �����ݷ���
//{
//  USART_SendData(USART6, data2);
//  while( USART_GetFlagStatus( USART6, USART_FLAG_TC ) != SET );
//}
//
//void SendToDraw2( uint8_t data8_H1, uint8_t data8_L1, uint8_t data8_H2, uint8_t data8_L2 , uint8_t data8_H3,uint8_t data8_L3,uint8_t data8_H4,uint8_t data8_L4) // ���ݸ���λ����ͼ
//{
//  //����ͷ
//  send_uchar( 0x03 );
//  send_uchar( 0xfc );
//  //����1
//  send_uchar( data8_H1 ); 
//  send_uchar( data8_L1 );
//   //����2
//  send_uchar( data8_H2 ); 
//  send_uchar( data8_L2 );
// 
//   send_uchar( data8_H3 ); 
//  send_uchar( data8_L3 );
//  
//   send_uchar( data8_H4 ); 
//  send_uchar( data8_L4);     
//   //����β
//  send_uchar( 0xfc );
//  send_uchar( 0x03 );
//}

//void USART3_IRQHandler(void) // USART3 �ж�
//{
//  // ����ģʽ�µ�USART3
//  if( USART_GetITStatus( USART3, USART_IT_RXNE ) != RESET ) 
//  {
//    ReceiveData = USART_ReceiveData(USART3); // ��������
////    USART_SendData( USART3, ReceiveData ); // ���ͽ��յ�������
//  }
//  
//  USART_ClearITPendingBit( USART3, USART_IT_RXNE );
//}

//void USART6_IRQHandler(void) // USART6 �ж�
//{
//  //����ģʽ�µ�USART6
//  if( USART_GetITStatus( USART6, USART_IT_RXNE ) != RESET ) 
//  { 
//    USART_ClearITPendingBit( USART6, USART_IT_RXNE );
//    ReceiveData_2 = USART_ReceiveData( USART6 ); // ��������
//
//    WirelessParaAdj( ReceiveData_2 ); // ���ߵ��� PID ��������
//    
//    USART_SendData( USART6, ReceiveData_2 ); //���ͽ��յ�������
//  }
//  USART_ClearITPendingBit( USART6, USART_IT_RXNE );
//}
