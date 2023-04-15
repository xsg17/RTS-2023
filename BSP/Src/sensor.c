#include "includes.h"
#include "sensor.h"
#include "motor.h"


void EXIT_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
        GPIO_InitTypeDef  GPIO_InitStructure;
 	
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
        RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 
	
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}


void EXTI2_IRQHandler(void)
{
	//delay_ms(1);
	if(EXTI_GetFlagStatus(EXTI_Line2) != RESET)
	{
	  step_speed = 0;
          EXTI_ClearITPendingBit(EXTI_Line2);
	}
	 //EXTI_ClearITPendingBit(EXTI_Line2);
}
