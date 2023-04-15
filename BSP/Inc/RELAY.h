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


#define PORT_RELAY						GPIOE
#define PORT_RELAY2						GPIOF

#define PIN_RELAY1						GPIO_Pin_0
#define PIN_RELAY2						GPIO_Pin_1
#define PIN_RELAY3						GPIO_Pin_2
#define PIN_RELAY4						GPIO_Pin_3
#define PIN_RELAY5						GPIO_Pin_4
#define PIN_RELAY6						GPIO_Pin_5
#define PIN_RELAY7						GPIO_Pin_6
#define PIN_RELAY8						GPIO_Pin_0      //F0
#define PIN_RELAY9						GPIO_Pin_1      //F1
#define PIN_RELAY10						GPIO_Pin_2      //F2

#define PIN_KEY1                                                 GPIO_Pin_7
#define PIN_KEY2                                                 GPIO_Pin_8
#define PIN_KEY3                                                 GPIO_Pin_9
#define PIN_KEY4                                                 GPIO_Pin_10
#define PIN_KEY5                                                 GPIO_Pin_11

#define RELAY_PORT_ENABLE()				{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);\
                                                         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);}

#define RELAY1_Toggle() 					GPIO_ToggleBits(PORT_RELAY, PIN_RELAY1)
#define RELAY1_OFF() 						GPIO_SetBits(PORT_RELAY, PIN_RELAY1)
#define RELAY1_ON() 						GPIO_ResetBits(PORT_RELAY, PIN_RELAY1)

#define RELAY2_Toggle() 					GPIO_ToggleBits(PORT_RELAY, PIN_RELAY2)
#define RELAY2_OFF() 						GPIO_SetBits(PORT_RELAY, PIN_RELAY2)
#define RELAY2_ON() 						GPIO_ResetBits(PORT_RELAY, PIN_RELAY2)

#define RELAY3_Toggle() 					GPIO_ToggleBits(PORT_RELAY, PIN_RELAY3)
#define RELAY3_OFF() 						GPIO_SetBits(PORT_RELAY, PIN_RELAY3)
#define RELAY3_ON() 						GPIO_ResetBits(PORT_RELAY, PIN_RELAY3)

#define RELAY4_Toggle() 					GPIO_ToggleBits(PORT_RELAY, PIN_RELAY4)
#define RELAY4_OFF() 						GPIO_SetBits(PORT_RELAY, PIN_RELAY4)
#define RELAY4_ON() 						GPIO_ResetBits(PORT_RELAY, PIN_RELAY4)

#define RELAY5_Toggle() 					GPIO_ToggleBits(PORT_RELAY, PIN_RELAY5)
#define RELAY5_OFF() 						GPIO_SetBits(PORT_RELAY, PIN_RELAY5)
#define RELAY5_ON() 						GPIO_ResetBits(PORT_RELAY, PIN_RELAY5)

#define RELAY6_Toggle() 					GPIO_ToggleBits(PORT_RELAY, PIN_RELAY6)
#define RELAY6_OFF() 						GPIO_SetBits(PORT_RELAY, PIN_RELAY6)
#define RELAY6_ON() 						GPIO_ResetBits(PORT_RELAY, PIN_RELAY6)

#define RELAY7_Toggle() 					GPIO_ToggleBits(PORT_RELAY, PIN_RELAY7)
#define RELAY7_OFF() 						GPIO_SetBits(PORT_RELAY, PIN_RELAY7)
#define RELAY7_ON() 						GPIO_ResetBits(PORT_RELAY, PIN_RELAY7)

#define RELAY8_Toggle() 					GPIO_ToggleBits(PORT_RELAY2, PIN_RELAY8)
#define RELAY8_OFF() 						GPIO_SetBits(PORT_RELAY2, PIN_RELAY8)
#define RELAY8_ON() 						GPIO_ResetBits(PORT_RELAY2, PIN_RELAY8)

#define RELAY9_Toggle() 					GPIO_ToggleBits(PORT_RELAY2, PIN_RELAY9)
#define RELAY9_OFF() 						GPIO_SetBits(PORT_RELAY2, PIN_RELAY9)
#define RELAY9_ON() 						GPIO_ResetBits(PORT_RELAY2, PIN_RELAY9)

#define RELAY10_Toggle() 					GPIO_ToggleBits(PORT_RELAY2, PIN_RELAY10)
#define RELAY10_OFF() 						GPIO_SetBits(PORT_RELAY2, PIN_RELAY10)
#define RELAY10_ON() 						GPIO_ResetBits(PORT_RELAY2, PIN_RELAY10)


void RELAY_Init (void);

extern unsigned char Key_Status1,Key_Status2,Key_Status3,Key_Status4,Key_Status5;




