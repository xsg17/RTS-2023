/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "includes.h"


#ifdef   ROBOT_DJ_MODULE
#define  ROBOT_DJ_EXT
#else
#define  ROBOT_DJ_EXT  extern
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
        int64_t Temparg;//���ֱ���
        int32_t Disparg;
        int32_t Exparg;//�����Ƕȣ� 157293.5=360��
        int16_t Expvel;//λ��ģʽ�µ������ٶ�
        int32_t postemp;
        int16_t posdel;
        int16_t Posflag;

}C620;




//  CAN 
#define	Motor_CAN               CAN1
#define Motor_CANSEND           CAN1_Send_Msg
#define Motor_CANSEND_h         CAN1_Send_Msg
#define Motor_CANSEND_Extend    CAN1_Send_Msg_ExtendID 

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void Motor_Analyze(CanRxMsg *canmsg);
void SetMotor (void);
void SetMotor_h (void);
void Motor_Speed_Ctrl_C620 (void);       //2021.4.9Ϊ������ͻ���޸�����Motor_Speed_Ctrl->Motor_Speed_Ctrl_C620
void Motor_Position_Ctrl(void);
void R2O_arc(float arc_r,int16_t speed);
void Position_Ctrl(void);
/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

extern char MotorData[8];
extern char MotorData_send[8][8]; // ���͸���λ��������
extern int16_t MotorVal[9][3];
extern int16_t posdel[9];
extern int use_t;
extern int16_t max_speed;
extern unsigned char flag_qickstop_fa;
extern unsigned char flag_qickstop_ti;
extern unsigned char flag_kick;

/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/