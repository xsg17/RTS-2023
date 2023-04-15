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

// CAN configuration
#define	ROBODRV_CAN        CAN2
#define ROBODRV_CANSEND    CAN2_Send_Msg

// Drive operation mode
#define PWM_MODE                            0x01
#define PWM_CURRENT_MODE                    0x02
#define PWM_VELOCITY_MODE                   0x03
#define PWM_POSITION_MODE                   0x04
#define PWM_VELOCITY_POSITION_MODE          0x05
#define CURRENT_VELOCITY_MODE               0x06
#define CURRENT_POSITION_MODE               0x07
#define CURRENT_VELOCITY_POSITION_MODE      0x08

// Driver information
//#define RBODRV_NUM    1

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void RoboDrive_Init (void);
void RoboDRV_Reset (unsigned char Group, unsigned char Number);
void RoboDRV_Mode_Choice (unsigned char Group, unsigned char Number, unsigned char Mode);
void RoboDRV_PWM_Velocity_Mode (unsigned char Group, unsigned char Number, short Temp_PWM, short Temp_Velocity);
void RoboDRV_PWM_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position);
void RoboDRV_Config (unsigned char Group, unsigned char Number, unsigned char Temp_Time);
void RoboDRV_DataAnalyze(CanRxMsg *canmsg);

/*
*********************************************************************************************************
*                                               MODULE END
*********************************************************************************************************
*/
  extern unsigned int current[5];
