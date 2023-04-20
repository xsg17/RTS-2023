/*          AUX     MD0
   ͨ��     0       0
   ����     0       1
   ����     1       1

*/
#include "includes.h"
#include "BT.h"
#define BT_MODULE
u8 res = 0;
// u8 USART3_TX_BUF[USART3_MAX_SEND_LEN];

// �ٶȲ��Ա���
float USE_TIME = 0; // �����ʱ����

u8 BT_Step = 0;
u8 BT_Count = 0;
u8 KEY_VALUE;

union
{
  int16_t result[6];
  char data[12];
} data1;

// ���ڳ�ʼ��
void BT_usart3_init()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // ʹ��GPIOBʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // ʹ��USART3ʱ��

  USART_DeInit(USART3); // ��λ����3

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // GPIOB11����ΪUSART3
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // GPIOB10����ΪUSART3

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; // GPIOB11��GPIOB10��ʼ��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             // ���ù���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        // �ٶ�50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           // ���츴�����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             // ����
  GPIO_Init(GPIOB, &GPIO_InitStructure);                   // ��ʼ��GPIOB11����GPIOB10

  USART_InitStructure.USART_BaudRate = 115200;                                    // ������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // �ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;                             // ����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // �շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);                                       // ��ʼ������3

  USART_Cmd(USART3, ENABLE); // ʹ�ܴ���

  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // �����ж�

  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // ��ռ���ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // �����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQͨ��ʹ��
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           // ����ϵͳ�ж����ȼ�����2
  NVIC_Init(&NVIC_InitStructure);                           // ����ָ���Ĳ�����ʼ��VIC�Ĵ���
}

// ��ʼ��Loraģ��GPIO
void LoRa_Init(void)
{
  //      GPIO_InitTypeDef  GPIO_InitStructure;
  ////      NVIC_InitTypeDef  NVIC_InitStructure;
  //
  //      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOC,GPIOFʱ��
  ////      RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
  //
  //      //AUX��MD0��������
  //
  //      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //MD0����
  //      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���ģʽ
  //      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  //      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  //      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  //      GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOE2,3,4
  //
  //      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //AUX����
  //      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  //      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  //      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  //      GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��
  //
  //     //����Ϊͨ��ģʽ
  //      AUX_OFF();
  //      MD0_OFF();

  BT_usart3_init(); // ��ʼ������3
}

// void usart1_init()
//{
//         NVIC_InitTypeDef NVIC_InitStructure;
//         GPIO_InitTypeDef GPIO_InitStructure;
//         USART_InitTypeDef USART_InitStructure;
//
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);      //ʹ��GPIOBʱ��
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);     //ʹ��USART3ʱ��
//
//  	USART_DeInit(USART1);                                     //��λ����3
//
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);  //GPIOB11����ΪUSART3
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);  //GPIOB10����ΪUSART3
//
//         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;  //GPIOB11��GPIOB10��ʼ��
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //�ٶ�50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //����
//	GPIO_Init(GPIOA,&GPIO_InitStructure);                     //��ʼ��GPIOB11����GPIOB10
//
//	USART_InitStructure.USART_BaudRate = 115200;                                    //������
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;                            //����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //�շ�ģʽ
//         USART_Init(USART1, &USART_InitStructure);                                      //��ʼ������3
//
//	USART_Cmd(USART1, ENABLE);                                                     //ʹ�ܴ���
//
//         USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);            //�����ж�
//
//
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�2
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//         NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
//	NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��VIC�Ĵ���
// }

// ͸�����䣨�����ã�
//     u8 Tran_Data[]={0};//͸������
// void LoRa_SendData1(void)
//{
//     sprintf((char*)Tran_Data,"0x01");
//     u3_printf("%s\r\n",Tran_Data);
// }
//
// void LoRa_SendData2(void)
//{
//
//     sprintf((char*)Tran_Data,"0x02");
//     u3_printf("%s\r\n",Tran_Data);
// }
//
//
// void u3_printf(char* fmt,...)
//{
//	u16 i,j;
//	va_list ap;
//	va_start(ap,fmt);
//	vsprintf((char*)USART3_TX_BUF,fmt,ap);
//	va_end(ap);
//	i=strlen((const char*)USART3_TX_BUF);                     //�˴η������ݵĳ���
//	for(j=0;j<i;j++)                                          //ѭ����������
//	{
//	  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);//�ȴ��ϴδ������
//		USART_SendData(USART3,(uint8_t)USART3_TX_BUF[j]); //�������ݵ�����3
//
//	}
//
//
// }

void USART3_IRQHandler(void)
{
  CPU_SR_ALLOC();

  CPU_CRITICAL_ENTER();
  OSIntEnter(); // Tell uC/OS-III that we are starting an ISR
  CPU_CRITICAL_EXIT();
  static union
  {
    uint8_t data[12];
    float LaserVal[3];

  } Laserposture;
  if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // ���յ�����
  {
    //               while((USART2->SR&0X40)==0);//�ȴ����ͽ���
    res = USART_ReceiveData(USART3);
    //                 USART_SendData(USART2,res);
    switch (BT_Step)
    {
    case 0:
      if (res == 0xbb)
      {
        BT_Step++;
        BT_Count = 0;
      }
      else
        BT_Step = 0;
      break;

    case 1:
      KEY_VALUE = res;
      BT_Step++;
      break;

    case 2:
      if (res == 0xee)
      {
        BT_Step = 0;
        BT_Count = 0;
      }
      else
        BT_Step = 0;
      break;

    default:
      BT_Step = 0;
      break;
    }
    //        while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//�ж��Ƿ������
  }
  USART_ClearITPendingBit(USART3, USART_IT_RXNE); // ���жϱ�־
  OSIntExit();                                    // Tell uC/OS-III that we are leaving the ISR
}

///*
//******************************************************************************
//*    �� �� ��    ��BT_Send_Char(u8 Char)
//*    ��    ��    ��ͨ����������һ���ַ�
//*    �������    ��
//*     pData      ���ַ����׵�ַ
//*      len       ���ַ�������
//******************************************************************************
//*/
void BT_Send_Char(u8 Char)
{
  USART_SendData(BT_USART, Char);
  // �ȴ��������һ���ֽ�
  while (USART_GetFlagStatus(BT_USART, USART_FLAG_TXE) != SET)
    ;
}
//
void TestBt_Send_Float(float senddata)
{
  char *add;               // ��ַ��������������data�ĵ�ַ
  add = (char *)&senddata; // Ҫ��(unsigned char *)
  for (uint8_t i = 0; i < 4; i++)
  {
    BT_Send_Char(*(add + i));
  }
}
//
/*
*********************************************************************************************************
* �� �� ��   ��send_oled(void)
* ��    ��   ������ʾ����������
**********************************
* printf("Laser_distance:%d\n",Laser_distance);
* printf("Laser_quality:%d\n",Laser_quality);
*********************************************************************************************************
*/
void send_oled(void)
{

  data1.result[0] = RoboMotion.curSpeed.Vx;
  data1.result[1] = RoboMotion.curSpeed.Vy;
  data1.result[2] = USE_TIME; // ��ʾʱ��
  data1.result[3] = RobotPos.pos_x;
  data1.result[4] = RobotPos.pos_y;
  data1.result[5] = RobotPos.zangle * 100;

  BT_Send_Char(0xaa); // ֡ͷ
  BT_Send_Char(data1.data[0]);
  BT_Send_Char(data1.data[1]);
  BT_Send_Char(data1.data[2]);
  BT_Send_Char(data1.data[3]);
  BT_Send_Char(data1.data[4]);
  BT_Send_Char(data1.data[5]);
  BT_Send_Char(data1.data[6]);
  BT_Send_Char(data1.data[7]);
  BT_Send_Char(data1.data[8]);
  BT_Send_Char(data1.data[9]);
  BT_Send_Char(data1.data[10]);
  BT_Send_Char(data1.data[11]);
  BT_Send_Char(0x01);
  BT_Send_Char(0xff);

  printf("RobotPos.pos_x:%3f\n", RobotPos.pos_x);
  printf("RobotPos.pos_y:%3f\n", RobotPos.pos_y);
  printf("RobotPos.zangle:%5f\n", RobotPos.zangle);
}

///*
//*********************************************************************************************************
//* Brief    : �ض��򴮿�
//*********************************************************************************************************
//*/
// int fputc( int ch, FILE *f )
//{
//  USART_SendData( USART2, ( uint8_t ) ch );
//  while( USART_GetFlagStatus( USART2, USART_FLAG_TXE ) != SET );
//  return( ch );
//}
