/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  RELAY_MODULE
#include "RELAY.h"

/*
*********************************************************************************************************
*                                            	DEFINES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
* Brief    : Initialize  all the LEDs on the board.
*
* Param(s) : none.
*
* Return(s): none.
*
*********************************************************************************************************
*/

void RELAY_Init (void) // LED≥ı ºªØ
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RELAY_PORT_ENABLE();
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = PIN_RELAY1; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = PIN_RELAY2; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = PIN_RELAY3; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = PIN_RELAY4; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_RELAY5; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_RELAY7; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_RELAY8; 
  GPIO_Init(PORT_RELAY2, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_RELAY9; 
  GPIO_Init(PORT_RELAY2, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_RELAY10; 
  GPIO_Init(PORT_RELAY2, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_RELAY6; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure); 
  
  
  
  RELAY1_OFF();
  RELAY2_OFF();
  RELAY3_OFF();
  RELAY4_OFF();
  RELAY5_OFF();
  RELAY6_OFF();
  RELAY7_OFF();
  RELAY8_OFF();
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  GPIO_InitStructure.GPIO_Pin = PIN_KEY1; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_KEY2; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_KEY3; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_KEY4; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = PIN_KEY5; 
  GPIO_Init(PORT_RELAY, &GPIO_InitStructure);
}

unsigned char Key_Status1=0,Key_Status2=0,Key_Status3=0,Key_Status4=0,Key_Status5=0;
void Key_Scan(void)
{
  Key_Status1=GPIO_ReadInputDataBit(PORT_RELAY,PIN_KEY1);
  Key_Status2=GPIO_ReadInputDataBit(PORT_RELAY,PIN_KEY2);
  Key_Status3=GPIO_ReadInputDataBit(PORT_RELAY,PIN_KEY3);
  Key_Status4=GPIO_ReadInputDataBit(PORT_RELAY,PIN_KEY4);
  Key_Status5=GPIO_ReadInputDataBit(PORT_RELAY,PIN_KEY5);
}













