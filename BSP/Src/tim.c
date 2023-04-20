/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  "tim.h"

/**********************************************************************************************************
*                                             LED_Init()
*   描述   : 初始化LED所用的IO口
**********************************************************************************************************/

void LEDF_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  LED_ENABLE;  //使能GPIOF时钟
  GPIO_Config;  //将IO口复用为定时器输出
  
  GPIO_InitStructure.GPIO_Pin = PIN_LED;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 复用输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
  GPIO_Init( GPIO_LED, &GPIO_InitStructure ); // 初始化
  
}

/**********************************************************************************************************
*                                               TIM_Init()
*   描述   : 初始化定时器配置
**********************************************************************************************************/

void TIM_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  TIM_ENABLE; // 使能定时器时钟
  
  TIM_TimeBaseInitStructure.TIM_Period = arr; // 自动重装载值
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc; // 定时器分频
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit( tim, &TIM_TimeBaseInitStructure ); // 初始化定时器
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 选择定时器模式:TIM脉冲宽度调制模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性:TIM输出比较极性低
  TIM_OC2Init( tim, &TIM_OCInitStructure );  // 根据指定的参数初始化外设TIM OC1
  
  TIM_OC2PreloadConfig( tim, TIM_OCPreload_Enable ); // 使能TIM在CCR1上的预装载寄存器
  TIM_ARRPreloadConfig( tim, ENABLE );//ARPE使能
  
  TIM_Cmd( tim, ENABLE ); // 使能定时器
}

void TIM2_Init(u16 f)//两个通道对应A板S和T
{
	TIM2_PORT_ENABLE();   //定时器端口初始化
	PORT_ENABLE();         //GPIO口初始化
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	GPIO_PinAFConfig(PORT_TIM2, GPIO_PinSource0, GPIO_AF_TIM2);//注意GPIO_Pin_和GPIO_PinSource的区别，后者复用时用到
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
