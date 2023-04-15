/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "includes.h"

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define PORT_LED1    GPIOF
#define PORT_LED2    GPIOE
#define PORT_PWR     GPIOH

// #define PIN_LED0    GPIO_Pin_9
#define PIN_LED1    GPIO_Pin_14
#define PIN_LED2    GPIO_Pin_11
#define PIN_PWR     GPIO_Pin_2

#define LED_PORT_ENABLE1()  { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOF, ENABLE ); }
#define LED_PORT_ENABLE2()  { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE ); }
#define PWR_PORT_ENABLE()  { RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOH, ENABLE ); }
//
//#define CORE_Toggle()    GPIO_ToggleBits(PORT_LED, PIN_LED0)
//#define CORE_ON()        GPIO_SetBits(PORT_LED, PIN_LED0)
//#define CORE_OFF()       GPIO_ResetBits(PORT_LED, PIN_LED0)
//
#define LED1_Toggle()    GPIO_ToggleBits(PORT_LED1, PIN_LED1)
#define LED1_OFF()        GPIO_SetBits(PORT_LED1, PIN_LED1)
#define LED1_ON()       GPIO_ResetBits(PORT_LED1, PIN_LED1)

#define LED2_Toggle()    GPIO_ToggleBits(PORT_LED2, PIN_LED2)
#define LED2_OFF()        GPIO_SetBits(PORT_LED2, PIN_LED2)
#define LED2_ON()       GPIO_ResetBits(PORT_LED2, PIN_LED2)

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void LED1_Init (void);
void LED2_Init (void);
void power_init(void);

/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/
