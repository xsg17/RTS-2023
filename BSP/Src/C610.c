/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#define ROBOT_DJ_MODULE_C610
#include "C610.h"
#include "PID.h"

/*
*********************************************************************************************************
*                                            	DEFINES
*********************************************************************************************************
*/

//  Kp      Ki       Kd     //速度环
int16_t posdel_C610[9];

extern C620 MOTOR[9];
char MotorData_C610[8];
char MotorData_send_C610[8][8];
int16_t MotorVal_C610[9][3];
int16_t max_speed_C610;
unsigned char flag_qickstop_fa_C610 = 0;
unsigned char flag_qickstop_ti_C610 = 0;
unsigned char flag_kick_C610 = 0;

/*
*********************************************************************************************************
* Brief    : 分析C610电调反馈的数据.
*
* Param(s) : data[0]:转子机械角度高8位
*            data[1]:转子机械角度低8位
*            data[2]:转子转速高8位
*            data[3]:转子转速低8位
*            data[4]:实际转矩电流高8位
*            data[5]:实际转矩电流低8位
*            data[6]:NULL
*            data[7]:NULL
*            发送频率：1KHz
*            转子机械角度值范围：0~8191（对应0~360°）
*            转子转速值单位：RPM
*
* Return(s): none.
*
**********************************************************************************************************
*/
void Motor_Analyze_C610(CanRxMsg *canmsg)
{
  static int i;
  char Motor_Id;

  Motor_Id = canmsg->StdId - 0x200;

  for (i = 0; i < canmsg->DLC; i++)
    MotorData_C610[i] = canmsg->Data[i];

  MotorVal_C610[Motor_Id][0] = MotorData_C610[0] << 8 | MotorData_C610[1]; // 转子机械角度
  MotorVal_C610[Motor_Id][1] = MotorData_C610[2] << 8 | MotorData_C610[3]; // 转子转速

  MotorVal_C610[Motor_Id][2] = MotorData_C610[4] << 8 | MotorData_C610[5]; // 转矩电流

  MOTOR_C610[Motor_Id].posdel = MotorVal_C610[Motor_Id][0] - MOTOR_C610[Motor_Id].CurPosition;

  if (MOTOR_C610[Motor_Id].CurPosition == 0)
  {
    MOTOR_C610[Motor_Id].posdel = 0;
  }

  if (MOTOR_C610[Motor_Id].posdel >= 4095 && MOTOR_C610[Motor_Id].CurPosition != 0)
  {
    MOTOR_C610[Motor_Id].posdel = MOTOR_C610[Motor_Id].posdel - 8192;
  }
  if (MOTOR_C610[Motor_Id].posdel <= -4095 && MOTOR_C610[Motor_Id].CurPosition != 0)
  {
    MOTOR_C610[Motor_Id].posdel = MOTOR_C610[Motor_Id].posdel + 8192;
  }

  MOTOR_C610[Motor_Id].Temparg += MOTOR_C610[Motor_Id].posdel;
  MotorData_send_C610[Motor_Id][2] = MotorData_C610[2];
  MotorData_send_C610[Motor_Id][3] = MotorData_C610[3];

  MOTOR_C610[Motor_Id].CurPosition = MotorVal_C610[Motor_Id][0];
  MOTOR_C610[Motor_Id].CurSpeed = MotorVal_C610[Motor_Id][1];
  MOTOR_C610[Motor_Id].ActCurrent = MotorVal_C610[Motor_Id][2];
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

void Motor_Speed_Ctrl_C610(void)
{
  MOTOR_C610[1].OutCurrent += PIDCal(&Motor1PID_C610, MOTOR_C610[1].ExpSpeed - MOTOR_C610[1].CurSpeed);
  MOTOR_C610[2].OutCurrent += PIDCal(&Motor2PID_C610, MOTOR_C610[2].ExpSpeed - MOTOR_C610[2].CurSpeed);
  MOTOR_C610[3].OutCurrent += PIDCal(&Motor3PID_C610, MOTOR_C610[3].ExpSpeed - MOTOR_C610[3].CurSpeed);
  MOTOR_C610[4].OutCurrent += PIDCal(&Motor4PID_C610, MOTOR_C610[4].ExpSpeed - MOTOR_C610[4].CurSpeed);
  MOTOR_C610[5].OutCurrent += PIDCal(&Motor5PID_C610, MOTOR_C610[5].ExpSpeed - MOTOR_C610[5].CurSpeed);
  MOTOR_C610[6].OutCurrent += PIDCal(&Motor6PID_C610, MOTOR_C610[6].ExpSpeed - MOTOR_C610[6].CurSpeed);
  MOTOR_C610[7].OutCurrent += PIDCal(&Motor7PID_C610, MOTOR_C610[7].ExpSpeed - MOTOR_C610[7].CurSpeed);
  MOTOR_C610[8].OutCurrent += PIDCal(&Motor8PID_C610, MOTOR_C610[8].ExpSpeed - MOTOR_C610[8].CurSpeed);
}

void Motor_Position_Ctrl_C610(void)
{
  if (abs(MOTOR_C610[1].Exparg - MOTOR_C610[1].Temparg) < 65500)
  {
    MOTOR_C610[1].ExpSpeed = PIDCal(&MotorPID1_C610, MOTOR_C610[1].Exparg - MOTOR_C610[1].Temparg);
  }
  else if (MOTOR_C610[1].Exparg > MOTOR_C610[1].Temparg)
  {
    MOTOR_C610[1].ExpSpeed = 1.05 * MOTOR_C610[1].Expvel;
  }
  else
  {
    MOTOR_C610[1].ExpSpeed = -1.05 * MOTOR_C610[1].Expvel;
  }

  if (abs(MOTOR_C610[2].Exparg - MOTOR_C610[2].Temparg) < 65500)
  {
    MOTOR_C610[2].ExpSpeed = PIDCal(&MotorPID2_C610, MOTOR_C610[2].Exparg - MOTOR_C610[2].Temparg);
  }
  else if (MOTOR_C610[2].Exparg > MOTOR_C610[2].Temparg)
  {
    MOTOR_C610[2].ExpSpeed = 1.05 * MOTOR_C610[2].Expvel;
  }
  else
  {
    MOTOR_C610[2].ExpSpeed = -1.05 * MOTOR_C610[2].Expvel;
  }

  if (abs(MOTOR_C610[3].Exparg - MOTOR_C610[3].Temparg) < 65500)
  {
    MOTOR_C610[3].ExpSpeed = PIDCal(&MotorPID3_C610, MOTOR_C610[3].Exparg - MOTOR_C610[3].Temparg);
  }
  else if (MOTOR_C610[3].Exparg > MOTOR_C610[3].Temparg)
  {
    MOTOR_C610[3].ExpSpeed = 1.05 * MOTOR_C610[3].Expvel;
  }
  else
  {
    MOTOR_C610[3].ExpSpeed = -1.05 * MOTOR_C610[3].Expvel;
  }

  if (abs(MOTOR_C610[4].Exparg - MOTOR_C610[4].Temparg) < 65500)
  {
    MOTOR_C610[4].ExpSpeed = PIDCal(&MotorPID4_C610, MOTOR_C610[4].Exparg - MOTOR_C610[4].Temparg);
  }
  else if (MOTOR_C610[4].Exparg > MOTOR_C610[4].Temparg)
  {
    MOTOR_C610[4].ExpSpeed = 1.05 * MOTOR_C610[4].Expvel;
  }
  else
  {
    MOTOR_C610[4].ExpSpeed = -1.05 * MOTOR_C610[4].Expvel;
  }

  if (abs(MOTOR_C610[5].Exparg - MOTOR_C610[5].Temparg) < 65500)
  {
    MOTOR_C610[5].ExpSpeed = PIDCal(&MotorPID5_C610, MOTOR_C610[5].Exparg - MOTOR_C610[5].Temparg);
  }
  else if (MOTOR_C610[5].Exparg > MOTOR_C610[5].Temparg)
  {
    MOTOR_C610[5].ExpSpeed = 1.05 * MOTOR_C610[5].Expvel;
  }
  else
  {
    MOTOR_C610[5].ExpSpeed = -1.05 * MOTOR_C610[5].Expvel;
  }

  if (abs(MOTOR_C610[6].Exparg - MOTOR_C610[6].Temparg) < 65500)
  {
    MOTOR_C610[6].ExpSpeed = PIDCal(&MotorPID6_C610, MOTOR_C610[6].Exparg - MOTOR_C610[6].Temparg);
  }
  else if (MOTOR_C610[6].Exparg > MOTOR_C610[6].Temparg)
  {
    MOTOR_C610[6].ExpSpeed = 1.05 * MOTOR_C610[6].Expvel;
  }
  else
  {
    MOTOR_C610[6].ExpSpeed = -1.05 * MOTOR_C610[6].Expvel;
  }

  if (abs(MOTOR_C610[7].Exparg - MOTOR_C610[7].Temparg) < 65500)
  {
    MOTOR_C610[7].ExpSpeed = PIDCal(&MotorPID7_C610, MOTOR_C610[7].Exparg - MOTOR_C610[7].Temparg);
  }
  else if (MOTOR_C610[7].Exparg > MOTOR_C610[7].Temparg)
  {
    MOTOR_C610[7].ExpSpeed = 1.05 * MOTOR_C610[7].Expvel;
  }
  else
  {
    MOTOR_C610[7].ExpSpeed = -1.05 * MOTOR_C610[7].Expvel;
  }

  if (abs(MOTOR_C610[8].Exparg - MOTOR_C610[8].Temparg) < 11000)
  {
    MOTOR_C610[8].ExpSpeed = PIDCal(&MotorPID8_C610, MOTOR_C610[8].Exparg - MOTOR_C610[8].Temparg);
  }
  else if (MOTOR_C610[8].Exparg > MOTOR_C610[8].Temparg)
  {
    MOTOR_C610[8].ExpSpeed = 1.05 * MOTOR_C610[8].Expvel;
  }
  else
  {
    MOTOR_C610[8].ExpSpeed = -1.05 * MOTOR_C610[8].Expvel;
  }
}

/*
*********************************************************************************************************
* Brief    : 向C610电调发送指令控制电流
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

void Position_Ctrl_C610(void)
{
}

void SetMotor_C610(void)
{
  u8 i;
  uint16_t can_id = 0x200;
  uint8_t Txmsg[8] = {0};

  for (i = 1; i <= 8; i++)
  {
    if (MOTOR_C610[i].OutCurrent > 16384)
    {
      MOTOR_C610[i].OutCurrent = 16384;
    }

    else if (MOTOR_C610[i].OutCurrent < -16384)
    {
      MOTOR_C610[i].OutCurrent = -16384;
    }
  }

  Txmsg[0] = (uint8_t)((MOTOR_C610[1].OutCurrent >> 8) & 0xFF);
  Txmsg[1] = (uint8_t)(MOTOR_C610[1].OutCurrent & 0xFF);
  Txmsg[2] = (uint8_t)((MOTOR_C610[2].OutCurrent >> 8) & 0xFF);
  Txmsg[3] = (uint8_t)(MOTOR_C610[2].OutCurrent & 0xFF);
  Txmsg[4] = (uint8_t)((MOTOR_C610[3].OutCurrent >> 8) & 0xFF);
  Txmsg[5] = (uint8_t)(MOTOR_C610[3].OutCurrent & 0xFF);
  Txmsg[6] = (uint8_t)((MOTOR_C610[4].OutCurrent >> 8) & 0xFF);
  Txmsg[7] = (uint8_t)(MOTOR_C610[4].OutCurrent & 0xFF);

  while (Motor_CANSEND_C610(can_id, Txmsg) == CAN_TxStatus_NoMailBox)
  {
  };
}

void SetMotor_h_C610(void) // 发高八位数据时使用
{
  u8 i;

  uint16_t can_id_h = 0x1FF;
  uint8_t Txmsg_h[8] = {0};
  for (i = 1; i <= 8; i++)
  {
    if (MOTOR_C610[i].OutCurrent > 16384)
    {
      MOTOR_C610[i].OutCurrent = 16384;
    }

    else if (MOTOR_C610[i].OutCurrent < -16384)
    {
      MOTOR_C610[i].OutCurrent = -16384;
    }
  }

  Txmsg_h[0] = (uint8_t)((MOTOR_C610[5].OutCurrent >> 8) & 0xFF);
  Txmsg_h[1] = (uint8_t)(MOTOR_C610[5].OutCurrent & 0xFF);
  Txmsg_h[2] = (uint8_t)((MOTOR_C610[6].OutCurrent >> 8) & 0xFF);
  Txmsg_h[3] = (uint8_t)(MOTOR_C610[6].OutCurrent & 0xFF);
  Txmsg_h[4] = (uint8_t)((MOTOR_C610[7].OutCurrent >> 8) & 0xFF);
  Txmsg_h[5] = (uint8_t)(MOTOR_C610[7].OutCurrent & 0xFF);
  Txmsg_h[6] = (uint8_t)((MOTOR_C610[8].OutCurrent >> 8) & 0xFF);
  Txmsg_h[7] = (uint8_t)(MOTOR_C610[8].OutCurrent & 0xFF);

  while (Motor_CANSEND_h_C610(can_id_h, Txmsg_h) == CAN_TxStatus_NoMailBox)
  {
  };
}

/***********************抓取机构*****************************************************/

int32_t initparg_2006 = 0;        // 2006初始角度为0,夹取机构处于闭合状态
int32_t initparg_3508 = 0;        // 3508初始角度为0，夹取机构处于闭合状态
int32_t openparg_2006 = 25000;    // 2006打开角度，夹环准备(25000)
int32_t overparg_3508 = -42000;   // 3508转动180，放环准备(-60000)
int32_t claparg_2006 = 1000;      // 2006夹环角度,根据环的个数进行调整(1000)
int32_t releaseparg_2006 = 45000; // 2006放环角度(53000)

int Capstep = 0; // 抓取步骤

/*******************************抓取机构函数**********************************/

void Pre_capture(void) // 夹环准备，打开之后向前怼一段距离，将环卡住
{
  MOTOR[5].Expvel = 2000;
  MOTOR[5].Exparg = initparg_3508;
  MOTOR[6].Expvel = 5000;
  MOTOR[6].Exparg = openparg_2006;
}
void Capture(void) // 抓取
{
  Capstep = 0;            // 抓取步骤
  MOTOR[5].Expvel = 1500; // 3508电机速度
  MOTOR[6].Expvel = 5000; // 2006电机速度
  while (Capstep < 3)
  {
    switch (Capstep)
    {
    case 0: // 夹环
      MOTOR[6].Exparg = claparg_2006;
      if (abs(MOTOR[6].Temparg - MOTOR[6].Exparg) <= 2000)
      {
        Capstep++;
      }
      // else
      //   Capstep = 0;
      break;
    case 1: // 将环夹稳之后，抓取机构翻转
      MOTOR[5].Exparg = overparg_3508;
      if (abs(MOTOR[5].Temparg - MOTOR[5].Exparg) <= 2000)
      {
        Capstep++;
      }
      // else
      //   Capstep = 1;
      break;
    case 2: // 将环放下
      MOTOR[6].Exparg = releaseparg_2006;
      Capstep++;
      break;
    }
  }
}

void Re_capture(void) // 环发射完成后，状态复原
{
  MOTOR[5].Expvel = 2000;
  MOTOR[6].Expvel = 5000; // 2006电机速度
  Capstep = 0;
  while (Capstep < 3)
  {
    switch (Capstep)
    {
    case 0:
      MOTOR[6].Exparg = initparg_2006;
      if (abs(MOTOR[6].Temparg - MOTOR[6].Exparg) <= 2000)
        Capstep++;
      break;
    case 1:
      MOTOR[5].Exparg = initparg_3508;
      if (abs(MOTOR[5].Temparg - MOTOR[5].Exparg) <= 2000)
        Capstep++;
      break;
    case 2:
      MOTOR[6].Exparg = openparg_2006;
      if (abs(MOTOR[6].Temparg - MOTOR[6].Exparg) <= 2000)
        Capstep++;
      break;

    default:
      break;
    }
  }
}

int Pos_flag_3508(void)
{ // 判断是否3508达到预期位置
  if (((MOTOR[5].Temparg - MOTOR[5].Exparg <= 2000) && (MOTOR[5].Temparg - MOTOR[5].Exparg >= 0)) || ((MOTOR[5].Exparg - MOTOR[5].Temparg <= 2000) && (MOTOR[5].Exparg - MOTOR[5].Temparg >= 0)))
    return 1;
  else
    return 0;
}

int Pos_flag_2006(void)
{ // 判断是否2006达到预期位置
  if (((MOTOR[6].Temparg - MOTOR[6].Exparg <= 600) && (MOTOR[6].Temparg - MOTOR[6].Exparg >= 0)) || ((MOTOR[6].Exparg - MOTOR[6].Temparg <= 600) && (MOTOR[6].Exparg - MOTOR[6].Temparg >= 0)))
    return 1;
  else
    return 0;
}

int Release_flag(void)
{ // 判断是否达到放环位置，确保稳定
  if (((MOTOR[5].Temparg - overparg_3508 <= 500) && (MOTOR[5].Temparg - overparg_3508 >= 0)) || ((overparg_3508 - MOTOR[5].Temparg <= 500) && (overparg_3508 - MOTOR[5].Temparg >= 0)))
  {
    if (((MOTOR[6].Temparg - claparg_2006 <= 600) && (MOTOR[6].Temparg - claparg_2006 >= 0)) || ((claparg_2006 - MOTOR[6].Temparg <= 600) && (claparg_2006 - MOTOR[6].Temparg >= 0)))
      return 1;
    else
      return 0;
  }
  else
    return 0;
}