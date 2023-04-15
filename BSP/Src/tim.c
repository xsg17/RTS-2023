/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  "tim.h"

/**********************************************************************************************************
*                                             LED_Init()
*   ����   : ��ʼ��LED���õ�IO��
**********************************************************************************************************/

void LEDF_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  LED_ENABLE;  //ʹ��GPIOFʱ��
  GPIO_Config;  //��IO�ڸ���Ϊ��ʱ�����
  
  GPIO_InitStructure.GPIO_Pin = PIN_LED;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // �������ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // �������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // ����
  GPIO_Init( GPIO_LED, &GPIO_InitStructure ); // ��ʼ��
  
}

/**********************************************************************************************************
*                                               TIM_Init()
*   ����   : ��ʼ����ʱ������
**********************************************************************************************************/

void TIM_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  TIM_ENABLE; // ʹ�ܶ�ʱ��ʱ��
  
  TIM_TimeBaseInitStructure.TIM_Period = arr; // �Զ���װ��ֵ
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc; // ��ʱ����Ƶ
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit( tim, &TIM_TimeBaseInitStructure ); // ��ʼ����ʱ��
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // �Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // �������:TIM����Ƚϼ��Ե�
  TIM_OC2Init( tim, &TIM_OCInitStructure );  // ����ָ���Ĳ�����ʼ������TIM OC1
  
  TIM_OC2PreloadConfig( tim, TIM_OCPreload_Enable ); // ʹ��TIM��CCR1�ϵ�Ԥװ�ؼĴ���
  TIM_ARRPreloadConfig( tim, ENABLE );//ARPEʹ��
  
  TIM_Cmd( tim, ENABLE ); // ʹ�ܶ�ʱ��
}

void TIM2_Init(u16 f)//����ͨ����ӦA��S��T
{
	TIM2_PORT_ENABLE();   //��ʱ���˿ڳ�ʼ��
	PORT_ENABLE();         //GPIO�ڳ�ʼ��
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	GPIO_PinAFConfig(PORT_TIM2, GPIO_PinSource0, GPIO_AF_TIM2);//ע��GPIO_Pin_��GPIO_PinSource�����𣬺��߸���ʱ�õ�
	GPIO_PinAFConfig(PORT_TIM2, GPIO_PinSource1, GPIO_AF_TIM2);
        
	GPIO_InitStruct.GPIO_Pin = PIN_TIM2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(PORT_TIM2, &GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 1000000/f - 1;	
	TIM_TimeBaseInitStruct.TIM_Prescaler = 90 - 1;    
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStruct);
	TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
    
}

void TIM4_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 1000000/50 - 1;		
	TIM_TimeBaseInitStruct.TIM_Prescaler = 90 - 1;    
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM4, &TIM_OCInitStruct);
	TIM_OC2Init(TIM4, &TIM_OCInitStruct);
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}
