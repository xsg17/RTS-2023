/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  BEEP_MODULE
#include "beep.h"

/*
*********************************************************************************************************
*                                            	DEFINES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
* Brief    : Initialize  all the beep on the board.
*
* Param(s) : none.
*
* Return(s): none.
*
*********************************************************************************************************
*/

void BEEP_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  BEEP_PORT_ENABLE();
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  
  GPIO_InitStructure.GPIO_Pin = PIN_BEEP; 
  GPIO_Init( PORT_BEEP, &GPIO_InitStructure );                              
}
