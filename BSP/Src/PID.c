/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  PID_MODULE
#include "PID.h"
#include "RobotCtrl.h"
#include "motor.h"

/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

extern Point error, present, target;
extern Velocity world;
extern volatile float target_position, current_position;

PIDType X = {1.2,1, 0.7, 0, 0, 10, 0, 0};
PIDType Y = {1.2,1, 0.7, 0, 0, 10, 0, 0};
PIDType W = {5,1, 0.7, 0, 0, 0.02, 0, 0};
PIDType Stepper_Motor = {10, 4, 2, 0, 0, 0.2, 0};
//                    Kp     Ki    Kd
//底盘PID参数 1 速度 3000
PIDType VelxPID1  = { 1.055, 3.425, 0  , 0, 0, 0, 0  }; // <----------> X USE THIS PID PARAMETER
PIDType VelyPID1  = { 1.055, 3.425, 0  , 0, 0, 0, 0  }; // <----------> Y USE THIS PID PARAMETER
PIDType ThetaPID1 = { 2.8, 1.50 , 1.0, 0, 0, 0, 0  }; // <----------> Z USE THIS PID PARAMETER
//底盘PID参数 2 速度 0 -- 200
PIDType VelxPID2  = { 0.775, 1.825, 0  , 0, 0, 0, 0  }; // <----------> X USE THIS PID PARAMETER
PIDType VelyPID2  = { 0.775, 1.825, 0  , 0, 0, 0, 0  }; // <----------> Y USE THIS PID PARAMETER
PIDType ThetaPID2 = { 1.00 , 0.70, 1.0, 0, 0, 0, 0  }; // <----------> Z USE THIS PID PARAMETER
  //ThetaPID2 former i =0.78
// ????? 什么DOG东西 ?????
//PIDType Velx_1PID = { 2.8, 0.10, 0, 0, 0, 0, 0 };
//PIDType Vely_1PID = { 2.20, 0.05, 0, 0, 0, 0, 0 };
// 位置PID参数
PIDType posxPID = {1.7, 0.6, 0.5, 0, 0, 0, 0};
PIDType posyPID = {1.7, 0.6, 0.5, 0, 0, 0, 0};//300,450,900,990      原始:1.5 0.6

/*
*********************************************************************************************************
*                                        fuzzy control    模糊控制
*********************************************************************************************************
*/
int qValue[2] = {0,0};
float qValueK[3];
int msE  = 0;
int msEC = 0;
extern float K1 = 0.03;
extern float K2 = 0.009;
extern float K3 = 0.001;
extern float maximum = 3000;
extern float minimum = -3000;

int ruleKp[7][7] = {3,3,2,2,1,0,0,
                    3,3,2,1,1,0,-1,
                    2,2,2,1,0,-1,-1,
                    2,2,1,0,-1,-2,-2,
                    1,1,0,-1,-1,-2,-2,
                    1,0,-1,-2,-2,-2,-3,
                    0,0,-2,-2,-2,-3,-3};
int ruleKi[7][7] = {-3,-3,-2,-2,-1,0,0,
                    -3,-3,-2,-1,-1,0,0,
                    -3,-2,-1,-1,0,1,1,
                    -2,-2,-1,0,1,2,2,
                    -2,-1,0,1,1,2,3,
                    0,0,1,1,2,3,3,
                    0,0,1,2,2,3,3};
int ruleKd[7][7] = {1,-1,-3,-3,-3,-2,1,
                    1,-1,-3,-2,-2,-1,0,
                    0,-1,-2,-2,-1,-1,0,
                    0,-1,-1,-1,-1,-1,0,
                    0,0,0,0,0,0,0,
                    3,-1,1,1,1,1,3,
                    3,2,2,2,1,1,3};

/*
*********************************************************************************************************
*                                      		 LOCAL VARIABLES
*********************************************************************************************************
*/

float PIDCal ( PIDType *PIDptr, float ThisError ) // 增量式PID
{
  float pError,dError,iError,temp,integral;	
  
  pError = ThisError - PIDptr->LastErr; 
  iError = ThisError;
  dError = ThisError - 2 * ( PIDptr->LastErr ) + PIDptr->PreErr;
  integral = PIDptr->KI * iError;
  
  if( ( integral > PIDptr->KIlimit ) && ( PIDptr->KIlimit != 0 ) )
    integral = PIDptr->KIlimit;
  
  else if( ( integral < -PIDptr->KIlimit ) && ( PIDptr->KIlimit != 0 ) )
    integral = -PIDptr->KIlimit;
  
  if( ( ThisError > PIDptr->delErr ) || ( ThisError < -PIDptr->delErr ) )	
    temp = PIDptr->KP * pError + integral + PIDptr->KD * dError;  
  
  else
    temp = 0;
  
  PIDptr->PreErr = PIDptr->LastErr;
  PIDptr->LastErr = ThisError;
  
  return temp;
}

float PIDCal_Fuzzy ( PIDType *PIDptr, float ThisError ) // 模糊增量式PID
{
  float pError,dError,iError,temp,integral;	
  
  pError = ThisError - PIDptr->LastErr; 
  iError = ThisError;
  dError = ThisError - 2 * ( PIDptr->LastErr ) + PIDptr->PreErr;
  /*********模糊控制************/
  
  qValue[0]=6.0*ThisError/(maximum-minimum);
  qValue[1]=3.0*pError/(maximum-minimum);
  
  msE  = qValue[0];
  msEC = qValue[1];
  if(msE>3)
  {
    msE = 3;
  }
  if(msE<-3)
  {
    msE = -3;
  }
  if(msEC>3)
  {
    msE = 3;
  }
  if(msEC<-3)
  {
    msE = -3;
  }
  
  qValueK[0] = abs(ruleKp[msE + 3][msEC + 3] * K1);
  qValueK[1] = abs(ruleKi[msE + 3][msEC + 3] * K2);
  qValueK[2] = abs(ruleKd[msE + 3][msEC + 3] * K3);
  
  
  integral = (PIDptr->KI + qValueK[1])* iError;
  
  if( ( integral > PIDptr->KIlimit ) && ( PIDptr->KIlimit != 0 ) )
    integral = PIDptr->KIlimit;
  
  else if( ( integral < -PIDptr->KIlimit ) && ( PIDptr->KIlimit != 0 ) )
    integral = -PIDptr->KIlimit;
  
  if( ( ThisError > PIDptr->delErr ) || ( ThisError < -PIDptr->delErr ) )	
    temp = (PIDptr->KP + qValueK[0]) * pError + integral + (PIDptr->KD + qValueK[2]) * dError;  
  
  else
    temp = 0;
  
  PIDptr->PreErr = PIDptr->LastErr;
  PIDptr->LastErr = ThisError;
  
  return temp;
}

float PIDCal_pos ( PIDType *PIDptr, float ThisError ) // 位置式PID
{
  float pError,dError,iError,temp,integral;	
  
  pError = ThisError ; 
  iError = ThisError + PIDptr->LastErr + PIDptr->PreErr;
  dError = ThisError -  PIDptr->LastErr ;
  
  integral = PIDptr->KI * iError;
  
  if( ( integral > PIDptr->KIlimit ) && ( PIDptr->KIlimit != 0 ) )
    integral = PIDptr->KIlimit;
  
  else if( ( integral < -PIDptr->KIlimit ) && ( PIDptr->KIlimit != 0 ) )
    integral = -PIDptr->KIlimit;
  
  if( ( ThisError > PIDptr->delErr ) || ( ThisError < -PIDptr->delErr ) )	
    temp = PIDptr->KP * pError + integral + PIDptr->KD * dError;  
  
  else
    temp = 0;
  
  PIDptr->PreErr = PIDptr->LastErr;
  PIDptr->LastErr = ThisError;
  
  if( abs( ThisError ) < 100 )
    temp = integral * 1.3 + PIDptr->KP * pError * 0.4;
  if( abs( ThisError ) > 100 && abs( ThisError ) < 500 )
    temp = integral * 1.0 + PIDptr->KP * pError * 0.6;
  if( abs( ThisError ) > 500 && abs( ThisError ) < 1000 )
    temp = integral * 0.5 + PIDptr->KP * pError * 1.0;
  if( abs( ThisError ) > 1000 )
    temp = integral * 0.25 + PIDptr->KP * pError * 1.4;
//  if(temp <= 1800)
//  {
//    temp = temp *0.6;
//  }
  return temp;
}

void Position_PID(PIDType PID_X, PIDType PID_Y, PIDType PID_Z)
{ PID_X.PreErr = target.x- present.x;
  PID_Y.PreErr = target.y- present.y;
  PID_Z.PreErr = target.z- present.z;
  
//  if((PID_X.SumErr > 50000) || (PID_Y.SumErr > 50000) || (PID_Z.SumErr > 50000))
//  {
//    PID_X.SumErr = PID_Y.SumErr =PID_Z.SumErr =50000;
//  }

  
  if(PID_X.PreErr <= PID_X.delErr)
  {
    world.Vy = 0;
  }
  else
  {
    if(PID_X.KP * PID_X.PreErr + PID_X.KI * PID_X.SumErr + PID_X.KD * (PID_X.PreErr - PID_X.LastErr) >= 500)
    {
      world.Vy = -500;
    }
    else
    {
      world.Vy = (PID_X.KP * PID_X.PreErr + PID_X.KI * PID_X.SumErr + PID_X.KD * (PID_X.PreErr - PID_X.LastErr))*-1;
    }
     
  }
  if(PID_Y.PreErr <= PID_Y.delErr)
  {
    world.Vx = 0;
  }
  else
  {
    if(PID_Y.KP * PID_Y.PreErr + PID_Y.KI * PID_Y.SumErr + PID_Y.KD * (PID_Y.PreErr - PID_Y.LastErr) >= 500)
    {
      world.Vx = 500;
    }
    else
    {
      world.Vx = PID_Y.KP * PID_Y.PreErr + PID_Y.KI * PID_Y.SumErr + PID_Y.KD * (PID_Y.PreErr - PID_Y.LastErr);
    }
     
  }
  if(PID_Z.PreErr <= PID_Z.delErr)
  {
    world.W = 0;
  }
  else
  {
    if(PID_Z.KP * PID_Z.PreErr + PID_Z.KI * PID_Z.SumErr + PID_Z.KD * (PID_Z.PreErr - PID_Z.LastErr) >=500)
    {
      world.W = -500;
    }
    else
    {
      world.W = (PID_Z.KP * PID_Z.PreErr + PID_Z.KI * PID_Z.SumErr + PID_Z.KD * (PID_Z.PreErr - PID_Z.LastErr))*-1; 
    }
  }
  
  PID_X.LastErr = PID_X.PreErr;
  PID_Y.LastErr = PID_Y.PreErr;
  PID_Z.LastErr = PID_Z.PreErr;
  PID_X.SumErr += PID_X.PreErr;
  PID_Y.SumErr += PID_Y.PreErr;
  PID_Z.SumErr += PID_Z.PreErr;
 } 
  
int Stepper_Motor_PID(PIDType PID)
{
  PID.PreErr = target_position - current_position;
  float speed = 0;
  
  if(fabs(PID.PreErr) <= fabs(PID.delErr))
  {
    speed = 0;
  }
  else
  {
    if(speed <= MAX_SPEED)
    {
       if(PID.KP * PID.PreErr + PID.KI * PID.SumErr + PID.KD * (PID.PreErr - PID.LastErr) >= ACCEL_RATE/TIM_ARR)
       {
         speed += ACCEL_RATE/TIM_ARR;
       }
       else
       {
         if(PID.KP * PID.PreErr + PID.KI * PID.SumErr + PID.KD * (PID.PreErr - PID.LastErr)<0){
         speed = fabs(PID.KP * PID.PreErr + PID.KI * PID.SumErr + PID.KD * (PID.PreErr - PID.LastErr));
         }
         else
         {
           speed = PID.KP * PID.PreErr + PID.KI * PID.SumErr + PID.KD * (PID.PreErr - PID.LastErr);
         }
       }
    }
    else{
      speed == MAX_SPEED;
    } 
  }
  return speed; 
}