/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "includes.h"

/*
*********************************************************************************************************
*                                              DEFINE
*********************************************************************************************************
*/
#define LED_ENABLE         { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE ); }
#define GPIO_LED            GPIOE
#define PIN_LED             GPIO_Pin_6
#define GPIO_Config        { GPIO_PinAFConfig( GPIOE, GPIO_PinSource6, GPIO_AF_TIM9 ); }

#define TIM_ENABLE         {RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);}
#define arr                 2000 - 1
#define psc                 1680 - 1

#define tim                 TIM9
#define tim_IRQn            TIM1_BRK_TIM9_IRQn
#define tim_IRQHandler      TIM1_BRK_TIM9_IRQHandler

#define PIN_TIM2            GPIO_Pin_0|GPIO_Pin_1
#define PORT_TIM2            GPIOA


#define TIM2_PORT_ENABLE()         {RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);}
#define PORT_ENABLE()          { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE ); }

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void LEDF_Init(void);
void TIM_Init(void);
void TIM2_Init(u16 f);
void TIM4_init(void);
/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/
