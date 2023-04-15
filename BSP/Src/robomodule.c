/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  ROBOMODULE_MODULE
#include "robomodule.h"

/*
*********************************************************************************************************
*                                             	 DEFINES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
* Brief    : 驱动程序初始化
*
* Param(s) : none.
*
* Return(s): none.
*
*********************************************************************************************************
*/

void RoboDrive_Init (void)
{
  RoboDRV_Reset(0, 0);
  delay_ms(500);
  
  RoboDRV_Mode_Choice(0, 0, PWM_VELOCITY_MODE);
  delay_ms(500);
  
  RoboDRV_Config (0, 1, 1);
  delay_ms(500);
  RoboDRV_Config (0, 2, 1);
  delay_ms(500);
  RoboDRV_Config (0, 3, 1);
  delay_ms(500);
  RoboDRV_Config (0, 4, 1);   
  delay_ms(500);
}

/*
*********************************************************************************************************
* Brief    : 驱动器复位指令
*
* Param(s) : Group  - Driver's CAN group.
*            Number - Driver's ID.
*
* Return(s): none.
*
*********************************************************************************************************
*/
void RoboDRV_Reset (unsigned char Group, unsigned char Number)
{
  uint16_t can_id = 0x000;
  uint8_t Txmsg[8];
  
  switch(Group)
  {
  case 0x0:
  case 0x1:
  case 0x2:
  case 0x3:
  case 0x4:
  case 0x5:
  case 0x6:
  case 0x7: can_id |= Group<<8; break;
  default: return;
  }
  
  switch(Number)
  {
  case 0x0:
  case 0x1:
  case 0x2:
  case 0x3:
  case 0x4:
  case 0x5:
  case 0x6:
  case 0x7:
  case 0x8:
  case 0x9:
  case 0xA:
  case 0xB:
  case 0xC:
  case 0xD:
  case 0xE:
  case 0xF: can_id |= Number<<4; break;
  default: return;
  }
  
  Txmsg[0] = 0x55;
  Txmsg[1] = 0x55;
  Txmsg[2] = 0x55;
  Txmsg[3] = 0x55;
  Txmsg[4] = 0x55;
  Txmsg[5] = 0x55;
  Txmsg[6] = 0x55;
  Txmsg[7] = 0x55;
  
  while(ROBODRV_CANSEND(can_id, Txmsg) == CAN_TxStatus_NoMailBox) {}
}

/*
*********************************************************************************************************
* Brief    : 驱动模式选择指令
*
* Param(s) : Group  - Driver's CAN group.
*            Number - Driver's ID.
*            Mode   - operation mode.
*
* Return(s): none.
*
*********************************************************************************************************
*/

void RoboDRV_Mode_Choice (unsigned char Group, unsigned char Number, unsigned char Mode)
{
  uint16_t can_id = 0x001;
  uint8_t Txmsg[8];
  
  switch(Group)
  {
  case 0x0:
  case 0x1:
  case 0x2:
  case 0x3:
  case 0x4:
  case 0x5:
  case 0x6:
  case 0x7: can_id |= Group<<8; break;
  default: return;
  }
  
  switch(Number)
  {
  case 0x0:
  case 0x1:
  case 0x2:
  case 0x3:
  case 0x4:
  case 0x5:
  case 0x6:
  case 0x7:
  case 0x8:
  case 0x9:
  case 0xA:
  case 0xB:
  case 0xC:
  case 0xD:
  case 0xE:
  case 0xF: can_id |= Number<<4; break;
  default: return;
  }
  
  Txmsg[0] = Mode;
  Txmsg[1] = 0x55;
  Txmsg[2] = 0x55;
  Txmsg[3] = 0x55;
  Txmsg[4] = 0x55;
  Txmsg[5] = 0x55;
  Txmsg[6] = 0x55;
  Txmsg[7] = 0x55;
  
  while(ROBODRV_CANSEND(can_id, Txmsg) == CAN_TxStatus_NoMailBox) {}
}

/*
*********************************************************************************************************
*                                  PWM速度模式下的数据指令
*        temp_pwm的取值范围如下：
*                                 0 ~ +5000
*
*        temp_velocity的取值范围如下：
*                                 0 ~ +32767
*********************************************************************************************************
*/

void RoboDRV_PWM_Velocity_Mode (unsigned char Group, unsigned char Number, short Temp_PWM, short Temp_Velocity)
{
  uint16_t can_id = 0x004;
  uint8_t Txmsg[8];
  
  switch(Group)
  {
  case 0x0:
  case 0x1:
  case 0x2:
  case 0x3:
  case 0x4:
  case 0x5:
  case 0x6:
  case 0x7: can_id |= Group<<8; break;
  default: return;
  }
  
  switch(Number)
  {
  case 0x0:
  case 0x1:
  case 0x2:
  case 0x3:
  case 0x4:
  case 0x5:
  case 0x6:
  case 0x7:
  case 0x8:
  case 0x9:
  case 0xA:
  case 0xB:
  case 0xC:
  case 0xD:
  case 0xE:
  case 0xF: can_id |= Number<<4; break;
  default: return;
  }
  
  if(Temp_PWM > 5000)
  {
    Temp_PWM = 5000;
  }
  else if(Temp_PWM < -5000)
  {
    Temp_PWM = -5000;
  }
  
  if(Temp_PWM < 0)
  {
    Temp_PWM = abs(Temp_PWM);
  }
  
  Txmsg[0] = (uint8_t)((Temp_PWM >> 8) & 0xFF);
  Txmsg[1] = (uint8_t)(Temp_PWM & 0xFF);
  Txmsg[2] = (uint8_t)((Temp_Velocity >> 8) & 0xFF);
  Txmsg[3] = (uint8_t)(Temp_Velocity & 0xFF);
  Txmsg[4] = 0x55;
  Txmsg[5] = 0x55;
  Txmsg[6] = 0x55;
  Txmsg[7] = 0x55;
  
  while(ROBODRV_CANSEND(can_id, Txmsg) == CAN_TxStatus_NoMailBox) {}
}

/*
*********************************************************************************************************
*                                  PWM速度位置模式下的数据指令
*           temp_pwm的取值范围如下：
*                                  0 ~ +5000
*           temp_velocity的取值范围如下：
*                                  0 ~ +32767
*           temp_position的取值范围如下：
*                                  32位有符号整型
*********************************************************************************************************
*/

void RoboDRV_PWM_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
  uint16_t can_id = 0x006;
  uint8_t Txmsg[8];
  
  switch(Group)
  {
  case 0x0:
  case 0x1:
  case 0x2:
  case 0x3:
  case 0x4:
  case 0x5:
  case 0x6:
  case 0x7: can_id |= Group<<8; break;
  default: return;
  }
  
  switch(Number)
  {
  case 0x0:
  case 0x1:
  case 0x2:
  case 0x3:
  case 0x4:
  case 0x5:
  case 0x6:
  case 0x7:
  case 0x8:
  case 0x9:
  case 0xA:
  case 0xB:
  case 0xC:
  case 0xD:
  case 0xE:
  case 0xF: can_id |= Number<<4; break;
  default: return;
  }
  
  if(Temp_PWM > 5000)
  {
    Temp_PWM = 5000;
  }
  else if(Temp_PWM < -5000)
  {
    Temp_PWM = -5000;
  }
  
  if(Temp_PWM < 0)
  {
    Temp_PWM = abs(Temp_PWM);
  }
  
  if(Temp_Velocity < 0)
  {
    Temp_Velocity = abs(Temp_Velocity);
  }
  
  Txmsg[0] = (unsigned char)((Temp_PWM>>8)&0xff);
  Txmsg[1] = (unsigned char)(Temp_PWM&0xff);
  Txmsg[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
  Txmsg[3] = (unsigned char)(Temp_Velocity&0xff);
  Txmsg[4] = (unsigned char)((Temp_Position>>24)&0xff);
  Txmsg[5] = (unsigned char)((Temp_Position>>16)&0xff);
  Txmsg[6] = (unsigned char)((Temp_Position>>8)&0xff);
  Txmsg[7] = (unsigned char)(Temp_Position&0xff);
  
  while(ROBODRV_CANSEND(can_id, Txmsg) == CAN_TxStatus_NoMailBox) {}
}

/*
*********************************************************************************************************
* Brief    : 驱动程序配置说明
*
* Param(s) : Group         - Driver's CAN group.
*            Number        - Driver's ID.
*            Temp_Time     - duration of feedback data( = Temp_Time * 10 ms)
* Return(s): none.
*********************************************************************************************************
*/

void RoboDRV_Config (unsigned char Group, unsigned char Number, unsigned char Temp_Time)
{
  uint16_t can_id = 0x00A;
  uint8_t Txmsg[8];
  
  switch(Group)
  {
  case 0x0:
  case 0x1:
  case 0x2:
  case 0x3:
  case 0x4:
  case 0x5:
  case 0x6:
  case 0x7: can_id |= Group<<8; break;
  default: return;
  }
  
  switch(Number)
  {
  case 0x1:
  case 0x2:
  case 0x3:
  case 0x4:
  case 0x5:
  case 0x6:
  case 0x7:
  case 0x8:
  case 0x9:
  case 0xA:
  case 0xB:
  case 0xC:
  case 0xD:
  case 0xE:
  case 0xF: can_id |= Number<<4; break;
  default: return;
  }
  
  Txmsg[0] = Temp_Time;
  Txmsg[1] = 0x00;
  Txmsg[2] = 0x55;
  Txmsg[3] = 0x55;
  Txmsg[4] = 0x55;
  Txmsg[5] = 0x55;
  Txmsg[6] = 0x55;
  Txmsg[7] = 0x55;
  
  while(ROBODRV_CANSEND(can_id, Txmsg) == CAN_TxStatus_NoMailBox) {}
}

/*
*********************************************************************************************************
*                                 驱动器反馈数据分析
*********************************************************************************************************
*/
unsigned int current[5];
void RoboDRV_DataAnalyze(CanRxMsg *canmsg)
{
  static int i;
  unsigned char Group,Number;
  char BrushDC_Data[8];
  
  Group = canmsg->StdId >>8;
  Number = canmsg->StdId >>4;
  
  for(i = 0; i < canmsg->DLC; i++)
    BrushDC_Data[i] = canmsg->Data[i];
  
  BrushDC.ID = Group<<4| Number;
  BrushDC.current  = (BrushDC_Data[0]<<8)|BrushDC_Data[1];
  BrushDC.velocity = (BrushDC_Data[2]<<8)|BrushDC_Data[3];
  BrushDC.position = ((BrushDC_Data[4]<<24)| (BrushDC_Data[5]<<16)| (BrushDC_Data[6]<<8)| BrushDC_Data[7]) - BrushDC.realposition;
  
  
  
  switch(Number)
  {
  case 0x1:
    current[1]= BrushDC.current;
      break;
  case 0x2:
     current[2]= BrushDC.current;
       break;
  case 0x3:
    current[3]= BrushDC.current;
    break;
  case 0x4:
    current[4]= BrushDC.current;
    
  case 0x5:
  case 0x6:
  case 0x7:
  case 0x8:
  case 0x9:
  case 0xA:
  case 0xB:
  case 0xC:
  case 0xD:
  case 0xE:
  case 0xF: break;
  default: return;
  }
  
  
  
  
  
}