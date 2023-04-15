/*          AUX     MD0
   通信     0       0
   配置     0       1
   升级     1       1

*/
#include "includes.h"
#include "BT.h"
#define  BT_MODULE
 u8 res=0;
//u8 USART3_TX_BUF[USART3_MAX_SEND_LEN];

// 速度测试变量
float USE_TIME = 0; // 任务计时变量

u8 BT_Step = 0;
u8 BT_Count = 0;
u8 KEY_VALUE;

union
{
  int16_t  result[6];
  char data[12];
}data1;

      //串口初始化
void BT_usart3_init()
{
        NVIC_InitTypeDef NVIC_InitStructure;
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);      //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);     //使能USART3时钟

 	USART_DeInit(USART3);                                     //复位串口3
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);  //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);  //GPIOB10复用为USART3	
	
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;  //GPIOB11和GPIOB10初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);                     //初始化GPIOB11，和GPIOB10
	
	USART_InitStructure.USART_BaudRate = 115200;                                    //波特率 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       	       //收发模式
        USART_Init(USART3, &USART_InitStructure);                                      //初始化串口3
 
	USART_Cmd(USART3, ENABLE);                                                     //使能串口 
	
        USART_ITConfig(USART3,USART_IT_RXNE, ENABLE);            //开启中断  
        
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化VIC寄存器
}

//初始化Lora模块GPIO
void LoRa_Init(void)
{

      
      BT_usart3_init();//初始化串口3
      
}



       
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
    
  }Laserposture;
	 if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)//接收到数据
	{	 
//               while((USART2->SR&0X40)==0);//等待发送结束
		res = USART_ReceiveData(USART3);
//                 USART_SendData(USART2,res);
                    switch( BT_Step )
                      {
                        case 0:
                        if( res == 0xbb )
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
                            if( res == 0xee )
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
//        while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//判断是否发送完成
              
             
        }
        USART_ClearITPendingBit(USART3,USART_IT_RXNE); // 清中断标志
        OSIntExit(); // Tell uC/OS-III that we are leaving the ISR 
 
}  

///*
//******************************************************************************
//*    函 数 名    ：BT_Send_Char(u8 Char)
//*    描    述    ：通过蓝牙发送一个字符
//*    输入参数    ：
//*     pData      ：字符串首地址
//*      len       ：字符串长度
//******************************************************************************
//*/
void BT_Send_Char(u8 Char)
{
  USART_SendData( BT_USART, Char );
  // 等待发送完成一个字节
  while(USART_GetFlagStatus( BT_USART, USART_FLAG_TXE ) != SET );
}
//
void TestBt_Send_Float( float senddata )
{
  char *add; // 地址变量，用来储存data的地址
  add = (char *)&senddata; // 要带(unsigned char *)
  for( uint8_t i = 0; i < 4; i++ )
  {
    BT_Send_Char( *( add + i ) );
  }
}
//
/*
*********************************************************************************************************
* 函 数 名   ：send_oled(void)
* 描    述   ：向显示屏发送数据
**********************************	
* printf("Laser_distance:%d\n",Laser_distance);
* printf("Laser_quality:%d\n",Laser_quality);
*********************************************************************************************************
*/
void send_oled(void)
{
  
  data1.result[0] = RoboMotion.curSpeed.Vx ;
  data1.result[1] = RoboMotion.curSpeed.Vy ;
  data1.result[2] = USE_TIME ; // 显示时间
  data1.result[3] = RobotPos.pos_x;
  data1.result[4] = RobotPos.pos_y;
  data1.result[5] = RobotPos.zangle*100;
  
  
  BT_Send_Char( 0xaa );//帧头
  BT_Send_Char( data1.data[0] );
  BT_Send_Char( data1.data[1] );
  BT_Send_Char( data1.data[2] );
  BT_Send_Char( data1.data[3] );
  BT_Send_Char( data1.data[4] );
  BT_Send_Char( data1.data[5] );
  BT_Send_Char( data1.data[6] );
  BT_Send_Char( data1.data[7] );
  BT_Send_Char( data1.data[8] );
  BT_Send_Char( data1.data[9] );
  BT_Send_Char( data1.data[10] );
  BT_Send_Char( data1.data[11] );
  BT_Send_Char( 0x01 );
  BT_Send_Char( 0xff );
  
  	printf("RobotPos.pos_x:%3f\n",RobotPos.pos_x);
  	printf("RobotPos.pos_y:%3f\n",RobotPos.pos_y);
  	printf("RobotPos.zangle:%5f\n",RobotPos.zangle);
}

///*
//*********************************************************************************************************
//* Brief    : 重定向串口
//*********************************************************************************************************
//*/
//int fputc( int ch, FILE *f )
//{
//  USART_SendData( USART2, ( uint8_t ) ch );
//  while( USART_GetFlagStatus( USART2, USART_FLAG_TXE ) != SET );
//  return( ch );
//}
