/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#define  ROBOT_DJ_MODULE
#include "C620.h"
#include "PID.h"
#include "vesc.h"


/*
*********************************************************************************************************
*                                            	DEFINES
*********************************************************************************************************
*/

//  Kp      Ki       Kd     //速度环
int16_t  posdel[9];
C620  	MOTOR[9];
PIDType Motor1PID =  {7, 0.7, 0,0,0,0,0};//舵轮OK，反应快，超调小
PIDType Motor2PID =  {7, 0.7, 0,0,0,0,0};
PIDType Motor3PID =  {7, 0.7, 0,0,0,0,0};
PIDType Motor4PID =  {7, 0.7, 0,0,0,0,0};

PIDType Motor5PID =  {7, 0.7, 0,0,0,0,0};//舵轮OK，反应快，超调小
PIDType Motor6PID =  {13, 0.72, 0,0,0,0,0};
PIDType Motor7PID =  {7, 0.7, 0,0,0,0,0};
PIDType Motor8PID =  {7, 0.7, 0,0,0,0,0};
//PIDType Motor5PID =  {17.5, 0.85,   0,0,0,0,0};
//PIDType Motor6PID =  {17.5, 0.85,   0,0,0,0,0};
//PIDType Motor7PID =  {17.5, 0.85,   0,0,0,0,0};
//PIDType Motor8PID =  {17.5, 0.85,   0,0,0,0,0};


//位置环
PIDType MotorPID1 =  {0.10, 0.10,   0,0,0,0,0};
PIDType MotorPID2 =  {0.10, 0.10,   0,0,0,0,0};
PIDType MotorPID3 =  {0.10, 0.10,   0,0,0,0,0};
PIDType MotorPID4 =  {0.10, 0.10,   0,0,0,0,0};

PIDType MotorPID5 =  {0.10, 0.10,   0,0,0,0,0};
PIDType MotorPID6 =  {0.3, 0.10,   0,0,0,0,0};
PIDType MotorPID7 =  {0.10, 0.10,   0,0,0,0,0};
PIDType MotorPID8 =  {0.10, 0.10,   0,0,0,0,0};
//PIDType MotorPID5 =  {10, 0.25,   0,0,0,15,0};
//PIDType MotorPID6 =  {10, 0.25,   0,0,0,15,0};
//PIDType MotorPID7 =  {10, 0.25,   0,0,0,15,0};
//PIDType MotorPID8 =  {10, 0.25,   0,0,0,15,0};

ROBOT_DJ_EXT C620      MOTOR[9];

char MotorData[8];
char MotorData_send[8][8];
int16_t MotorVal[9][3];
int16_t max_speed;
unsigned char flag_qickstop_fa = 0;
unsigned char flag_qickstop_ti = 0;
unsigned char flag_kick=0;
/*
*********************************************************************************************************
* Brief    : 分析C620电调反馈的数据.
*
* Param(s) : data[0]:转子机械角度高8位
*            data[1]:转子机械角度低8位
*            data[2]:转子转速高8位
*            data[3]:转子转速低8位
*            data[4]:实际转矩电流高8位
*            data[5]:实际转矩电流低8位
*            data[6]:电机温度
*            data[7]:NULL
*            发送频率：1KHz
*            转子机械角度值范围：0~8191（对应0~360°）
*            转子转速值单位：RPM
*            电机温度单位：℃
*
* Return(s): none.
*
**********************************************************************************************************
*/
void Motor_Analyze(CanRxMsg *canmsg)
{
  static int i;
  char Motor_Id;
  
  Motor_Id = canmsg->StdId - 0x200; 
  
  for(i = 0; i < canmsg->DLC; i++)
    MotorData[i] = canmsg->Data[i];
  
  MotorVal[Motor_Id][0]= MotorData[0]<<8 | MotorData[1];//转子机械角度
  MotorVal[Motor_Id][1]= MotorData[2]<<8 | MotorData[3];//转子转速
  
  MotorVal[Motor_Id][2]= MotorData[4]<<8 | MotorData[5];//转矩电流
  
  MOTOR[Motor_Id].posdel = MotorVal[Motor_Id][0]- MOTOR[Motor_Id].CurPosition;
  
  if(MOTOR[Motor_Id].CurPosition==0)
  {
    MOTOR[Motor_Id].posdel=0;
  }

  if(MOTOR[Motor_Id].posdel>=4095 && MOTOR[Motor_Id].CurPosition!=0) 
  {
    MOTOR[Motor_Id].posdel=MOTOR[Motor_Id].posdel-8192;
  }
  if(MOTOR[Motor_Id].posdel<=-4095 && MOTOR[Motor_Id].CurPosition!=0) 
  {
    MOTOR[Motor_Id].posdel=MOTOR[Motor_Id].posdel+8192;
  }
  
  MOTOR[Motor_Id].Temparg += MOTOR[Motor_Id].posdel;
  MotorData_send[Motor_Id][2]=MotorData[2];
  MotorData_send[Motor_Id][3]=MotorData[3];
  
  
  
  MOTOR[Motor_Id].CurPosition = MotorVal[Motor_Id][0];
  MOTOR[Motor_Id].CurSpeed = MotorVal[Motor_Id][1];
  MOTOR[Motor_Id].ActCurrent = MotorVal[Motor_Id][2];

}



/*
*********************************************************************************************************
* Brief    : 根据电调反馈的速度 用PID 做电流闭环
*
* Param(s) :
*
* Return(s): none.
*
*********************************************************************************************************
*/
/*void Motor_Speed_Ctrl (void)
{
  MOTOR[1].OutCurrent += PIDCal(&Motor1PID, MOTOR[1].ExpSpeed - MOTOR[1].CurSpeed);
  MOTOR[2].OutCurrent += PIDCal(&Motor2PID, MOTOR[2].ExpSpeed - MOTOR[2].CurSpeed);
  MOTOR[3].OutCurrent += PIDCal(&Motor3PID, MOTOR[3].ExpSpeed - MOTOR[3].CurSpeed);
  MOTOR[4].OutCurrent += PIDCal(&Motor4PID, MOTOR[4].ExpSpeed - MOTOR[4].CurSpeed);
  MOTOR[5].OutCurrent += PIDCal(&Motor5PID, MOTOR[5].ExpSpeed - MOTOR[5].CurSpeed);
  MOTOR[6].OutCurrent += PIDCal(&Motor6PID, MOTOR[6].ExpSpeed - MOTOR[6].CurSpeed);
  MOTOR[7].OutCurrent += PIDCal(&Motor7PID, MOTOR[7].ExpSpeed - MOTOR[7].CurSpeed);
  MOTOR[8].OutCurrent += PIDCal(&Motor8PID, MOTOR[8].ExpSpeed - MOTOR[8].CurSpeed);
}*/


void Motor_Speed_Ctrl_C620 (void)       //2021.4.9为消除冲突，修改名称Motor_Speed_Ctrl->Motor_Speed_Ctrl_C620
{
//  MOTOR[1].OutCurrent += PIDCal(&Motor1PID, Wheel[1].Target_V - MOTOR[1].CurSpeed);
//  MOTOR[2].OutCurrent += PIDCal(&Motor2PID, Wheel[2].Target_V - MOTOR[2].CurSpeed);
//  MOTOR[3].OutCurrent += PIDCal(&Motor3PID, Wheel[3].Target_V - MOTOR[3].CurSpeed);
//  MOTOR[4].OutCurrent += PIDCal(&Motor4PID, Wheel[4].Target_V - MOTOR[4].CurSpeed);
  MOTOR[1].OutCurrent += PIDCal(&Motor1PID, MOTOR[1].ExpSpeed - MOTOR[1].CurSpeed);
  MOTOR[2].OutCurrent += PIDCal(&Motor2PID, MOTOR[2].ExpSpeed - MOTOR[2].CurSpeed);
  MOTOR[3].OutCurrent += PIDCal(&Motor3PID, MOTOR[3].ExpSpeed - MOTOR[3].CurSpeed);
  MOTOR[4].OutCurrent += PIDCal(&Motor4PID, MOTOR[4].ExpSpeed - MOTOR[4].CurSpeed);
  MOTOR[5].OutCurrent += PIDCal(&Motor5PID, MOTOR[5].ExpSpeed - MOTOR[5].CurSpeed);
  MOTOR[6].OutCurrent += PIDCal(&Motor6PID, MOTOR[6].ExpSpeed - MOTOR[6].CurSpeed);
  MOTOR[7].OutCurrent += PIDCal(&Motor7PID, MOTOR[7].ExpSpeed - MOTOR[7].CurSpeed);
  MOTOR[8].OutCurrent += PIDCal(&Motor8PID, MOTOR[8].ExpSpeed - MOTOR[8].CurSpeed);
}



void Motor_Position_Ctrl(void)
{
    if(abs(MOTOR[1].Exparg - MOTOR[1].Temparg)<11000)
  {
    MOTOR[1].ExpSpeed=PIDCal(&MotorPID1, MOTOR[1].Exparg - MOTOR[1].Temparg);
  }
  else if(MOTOR[1].Exparg>MOTOR[1].Temparg)
  {
    MOTOR[1].ExpSpeed=1.05*MOTOR[1].Expvel;
  }
  else
  {
    MOTOR[1].ExpSpeed=-1.05*MOTOR[1].Expvel;
  }
  
  
  
  if(abs(MOTOR[2].Exparg - MOTOR[2].Temparg)<11000)
  {
    MOTOR[2].ExpSpeed=PIDCal(&MotorPID2, MOTOR[2].Exparg - MOTOR[2].Temparg);
  }
  else if(MOTOR[2].Exparg>MOTOR[2].Temparg)
  {
    MOTOR[2].ExpSpeed=1.05*MOTOR[2].Expvel;
  }
  else
  {
    MOTOR[2].ExpSpeed=-1.05*MOTOR[2].Expvel;
  }
  
  
  
  if(abs(MOTOR[3].Exparg - MOTOR[3].Temparg)<11000)
  {
    MOTOR[3].ExpSpeed=PIDCal(&MotorPID3, MOTOR[3].Exparg - MOTOR[3].Temparg);
  }
  else if(MOTOR[3].Exparg>MOTOR[3].Temparg)
  {
    MOTOR[3].ExpSpeed=1.05*MOTOR[3].Expvel;
  }
  else
  {
    MOTOR[3].ExpSpeed=-1.05*MOTOR[3].Expvel;
  }
  
  
  
  if(abs(MOTOR[4].Exparg - MOTOR[4].Temparg)<11000)
  {
    MOTOR[4].ExpSpeed=PIDCal(&MotorPID4, MOTOR[4].Exparg - MOTOR[4].Temparg);
  }
  else if(MOTOR[4].Exparg>MOTOR[4].Temparg)
  {
    MOTOR[4].ExpSpeed=1.05*MOTOR[4].Expvel;
  }
  else
  {
    MOTOR[4].ExpSpeed=-1.05*MOTOR[4].Expvel;
  }
  
  
  
  if(abs(MOTOR[5].Exparg - MOTOR[5].Temparg)<11000)
  {
    MOTOR[5].ExpSpeed= PIDCal(&MotorPID5, MOTOR[5].Exparg - MOTOR[5].Temparg);
  }
  else if(MOTOR[5].Exparg>MOTOR[5].Temparg)
  {
    MOTOR[5].ExpSpeed=1.05*MOTOR[5].Expvel;
  }
  else
  {
    MOTOR[5].ExpSpeed=-1.05*MOTOR[5].Expvel;
  }
  
  
  
  if(abs(MOTOR[6].Exparg - MOTOR[6].Temparg)<11000)
  {
    MOTOR[6].ExpSpeed=PIDCal(&MotorPID6, MOTOR[6].Exparg - MOTOR[6].Temparg);
  }
  else if(MOTOR[6].Exparg>MOTOR[6].Temparg)
  {
    MOTOR[6].ExpSpeed=1.05*MOTOR[6].Expvel;
  }
  else
  {
    MOTOR[6].ExpSpeed=-1.05*MOTOR[6].Expvel;
  }
  
  
  
  if(abs(MOTOR[7].Exparg - MOTOR[7].Temparg)<11000)
  {
    MOTOR[7].ExpSpeed=PIDCal(&MotorPID7, MOTOR[7].Exparg - MOTOR[7].Temparg);
  }
  else if(MOTOR[7].Exparg>MOTOR[7].Temparg)
  {
    MOTOR[7].ExpSpeed=1.05*MOTOR[7].Expvel;
  }
  else
  {
    MOTOR[7].ExpSpeed=-1.05*MOTOR[7].Expvel;
  }
  
  
  
  if(abs(MOTOR[8].Exparg - MOTOR[8].Temparg)<11000)
  {
    MOTOR[8].ExpSpeed=PIDCal(&MotorPID8, MOTOR[8].Exparg - MOTOR[8].Temparg);
  }
  else if(MOTOR[8].Exparg>MOTOR[8].Temparg)
  {
    MOTOR[8].ExpSpeed=1.05*MOTOR[8].Expvel;
  }
  else
  {
    MOTOR[8].ExpSpeed=-1.05*MOTOR[8].Expvel;
  }
  
}


/*
*********************************************************************************************************
* Brief    : 向C620电调发送指令控制电流
*
* Param(s) : can_id=0x200（电调ID：1~4）或0x1FF（电调ID：5~8）
*            控制电流值范围 -16384~16384  对应转矩电流范围 -20~20A
*
*            data[0]:控制电流高8位
*            data[1]:控制电流低8位
*            data[2]:控制电流高8位
*            data[3]:控制电流低8位
*            data[4]:控制电流高8位
*            data[5]:控制电流低8位
*            data[6]:控制电流高8位
*            data[7]:控制电流低8位
*
* Return(s): none.
*
*********************************************************************************************************
*/

void Position_Ctrl(void)
{
  
}

void SetMotor (void)
{
  u8 i;
  uint16_t can_id = 0x200;
  uint8_t Txmsg[8]={0};
  
  for(i=1;i<=8;i++)
  {
    if(MOTOR[i].OutCurrent > 16384){ MOTOR[i].OutCurrent = 16384; }
    
    else if(MOTOR[i].OutCurrent < -16384){ MOTOR[i].OutCurrent = -16384; }
  }
  
  Txmsg[0] = (uint8_t)(( MOTOR[1].OutCurrent >> 8 ) & 0xFF);
  Txmsg[1] = (uint8_t)(  MOTOR[1].OutCurrent & 0xFF) ;
  Txmsg[2] = (uint8_t)(( MOTOR[2].OutCurrent >> 8 ) & 0xFF);
  Txmsg[3] = (uint8_t)(  MOTOR[2].OutCurrent & 0xFF) ;
  Txmsg[4] = (uint8_t)(( MOTOR[3].OutCurrent >> 8 ) & 0xFF);
  Txmsg[5] = (uint8_t)(  MOTOR[3].OutCurrent & 0xFF) ;
  Txmsg[6] = (uint8_t)(( MOTOR[4].OutCurrent >> 8 ) & 0xFF);
  Txmsg[7] = (uint8_t)(  MOTOR[4].OutCurrent & 0xFF) ;
  
  while(Motor_CANSEND(can_id, Txmsg) == CAN_TxStatus_NoMailBox){};
  
}

void SetMotor_h (void) // 发高八位数据时使用
{
  u8 i;
  
  uint16_t can_id_h = 0x1FF;
  uint8_t Txmsg_h[8]={0};
  for(i=1;i<=8;i++)
  {
    if(MOTOR[i].OutCurrent > 16384){ MOTOR[i].OutCurrent = 16384; }
    
    else if(MOTOR[i].OutCurrent < -16384){ MOTOR[i].OutCurrent = -16384; }
  }
  
  Txmsg_h[0] = (uint8_t)(( MOTOR[5].OutCurrent >> 8 ) & 0xFF);
  Txmsg_h[1] = (uint8_t)(  MOTOR[5].OutCurrent & 0xFF) ;
  Txmsg_h[2] = (uint8_t)(( MOTOR[6].OutCurrent >> 8 ) & 0xFF);
  Txmsg_h[3] = (uint8_t)(  MOTOR[6].OutCurrent & 0xFF) ;
  Txmsg_h[4] = (uint8_t)(( MOTOR[7].OutCurrent >> 8 ) & 0xFF);
  Txmsg_h[5] = (uint8_t)(  MOTOR[7].OutCurrent & 0xFF) ;
  Txmsg_h[6] = (uint8_t)(( MOTOR[8].OutCurrent >> 8 ) & 0xFF);
  Txmsg_h[7] = (uint8_t)(  MOTOR[8].OutCurrent & 0xFF) ;
  
  while(Motor_CANSEND_h(can_id_h, Txmsg_h) == CAN_TxStatus_NoMailBox){};
}