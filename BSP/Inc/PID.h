/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "includes.h"

/*
*********************************************************************************************************
*                                             EXPORTED_TYPES
*********************************************************************************************************
*/

typedef struct
{
  float KP;
  float KI;
  float KD;
  float PreErr;
  float LastErr;
  float delErr;       // error tolerance
  float KIlimit;      // intergral limit
  float SumErr;
  
}PIDType;

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

float PIDCal( PIDType *PIDptr, float ThisError );
float PIDCal_Fuzzy ( PIDType *PIDptr, float ThisError );
float PIDCal_pos( PIDType *PIDptr, float ThisError );
void Position_PID(PIDType PID_X, PIDType PID_Y, PIDType PID_Z);
int Stepper_Motor_PID(PIDType PID);

// PID参数调节 1
extern PIDType VelxPID1;
extern PIDType VelyPID1;
extern PIDType ThetaPID1;
// PID参数调节 2
extern PIDType VelxPID2;
extern PIDType VelyPID2;
extern PIDType ThetaPID2;

extern float K1;
extern float K2;
extern float K3;
extern float maximum;
extern float minimum;

extern PIDType posxPID;
extern PIDType posyPID;
extern PIDType X;
extern PIDType Y;
extern PIDType W;
extern PIDType Stepper_Motor;
extern PIDType NavigationPID;
extern PIDType NavigationWPID;
extern PIDType NavigationTPID;
/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/
