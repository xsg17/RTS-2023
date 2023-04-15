/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  LED_MODULE
#include "led.h"

/*
*********************************************************************************************************
*                                            	DEFINES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
* Brief    : 初始化板上的所有LED
*
* Param(s) : none.
*
* Return(s): none.
*
*********************************************************************************************************
*/

void LED1_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  LED_PORT_ENABLE1();
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  //	GPIO_InitStructure.GPIO_Pin = PIN_LED0;
  //	GPIO_Init(PORT_LED, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_LED1;
  GPIO_Init(PORT_LED1, &GPIO_InitStructure);
  
  
  

}

void LED2_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  LED_PORT_ENABLE2();
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  //	GPIO_InitStructure.GPIO_Pin = PIN_LED0;
  //	GPIO_Init(PORT_LED, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_LED2;
  GPIO_Init(PORT_LED2, &GPIO_InitStructure);
  
}

void power_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  PWR_PORT_ENABLE();
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  //	GPIO_InitStructure.GPIO_Pin = PIN_LED0;
  //	GPIO_Init(PORT_LED, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_PWR;
  GPIO_Init(PORT_PWR, &GPIO_InitStructure);
  GPIO_SetBits(PORT_PWR, PIN_PWR);
}
