/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "includes.h"


#ifdef   ROBOT_DJ_MODULE_C610
#define  ROBOT_DJ_EXT_C610
#else
#define  ROBOT_DJ_EXT_C610  extern
#endif
/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/


typedef struct
{
	int16_t CurSpeed;      // (RPM, RPM, 20/16384 A)
	int16_t ExpSpeed;
	int16_t ActCurrent;
	int16_t OutCurrent;
        int16_t CurPosition;
        int16_t ExpPosition;
        int64_t Temparg;//积分变量,当前角度
        int32_t Disparg;
        int32_t Exparg;//期望角度， 157293.5=360°
        int16_t Expvel;//位置模式下的期望速度
        int32_t postemp;
        int16_t posdel;
        int16_t Posflag;

}C610;

/*
int32_t initparg_2006=0;//2006初始角度为0,夹取机构处于闭合状态
int32_t initparg_3508=0;//3508初始角度为0，夹取机构处于闭合状态
int32_t overturn_3508=79000;//3508转动180，机构翻转
int32_t openparg_2006=120000;//2006打开角度
int32_t claparg_2006=5000;//2006夹环角度,根据环的个数进行调整
int32_t releaseparg_2006=140000;//2006放环角度
*/

static  uint8_t Capture_sign=0;//抓取标志状态


ROBOT_DJ_EXT_C610  C610  	  MOTOR_C610[9];

//  CAN 配置
#define	Motor_CAN_C610               CAN2
#define Motor_CANSEND_C610          CAN2_Send_Msg
#define Motor_CANSEND_h_C610         CAN2_Send_Msg
/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void Motor_Analyze_C610(CanRxMsg *canmsg);
void SetMotor_C610 (void);
void SetMotor_C610_h (void);
void Motor_Speed_Ctrl_C610 (void);       //2021.4.9为消除冲突，修改名称Motor_Speed_Ctrl->Motor_Speed_Ctrl_C620
void Motor_Position_Ctrl_C610(void);
void R2O_arc_C610(float arc_r,int16_t speed);
void Position_Ctrl_C610(void);


void Pre_capture(void);//抓取准备
void Capture(void); //抓取机构，2006，3508同时控制
void Release_capture(void);//抓取机构翻转后，将环放下
void Re_capture(void);//复原抓取机构位置
int Pos_flag_2006(void);//2006电机位置标志
int Pos_flag_3508(void);//3508电机位置标志
int Release_flag(void);//放环标志
//void tese3508(void);
/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

extern char MotorData_C610[8];
extern char MotorData_send_C610[8][8]; // 传送给上位机的数据
extern int16_t MotorVal_C610[9][3];
extern int16_t posdel_C610[9];
extern int use_t_C610;
extern int16_t max_speed_C610;
extern unsigned char flag_qickstop_fa_C610;
extern unsigned char flag_qickstop_ti_C610;
extern unsigned char flag_kick_C610;

/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/