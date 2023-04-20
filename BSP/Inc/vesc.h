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




#define CAN_STATUS_MSGS_TO_STORE	10  



/*vesc中CAN命令
*********************************************************************************************************
*/
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS,
	CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_STATUS_2,
	CAN_PACKET_STATUS_3,
	CAN_PACKET_STATUS_4,
	CAN_PACKET_PING,
	CAN_PACKET_PONG,
	CAN_PACKET_DETECT_APPLY_ALL_FOC,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
	CAN_PACKET_CONF_CURRENT_LIMITS,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_FOC_ERPMS,
	CAN_PACKET_CONF_STORE_FOC_ERPMS,
	CAN_PACKET_STATUS_5,
	CAN_PACKET_POLL_TS5700N8501_STATUS,
	CAN_PACKET_CONF_BATTERY_CUT,
	CAN_PACKET_CONF_STORE_BATTERY_CUT,
	CAN_PACKET_SHUTDOWN,
	CAN_PACKET_IO_BOARD_ADC_1_TO_4,
	CAN_PACKET_IO_BOARD_ADC_5_TO_8,
	CAN_PACKET_IO_BOARD_ADC_9_TO_12,
	CAN_PACKET_IO_BOARD_DIGITAL_IN,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM,
	CAN_PACKET_BMS_V_TOT,
	CAN_PACKET_BMS_I,
	CAN_PACKET_BMS_AH_WH,
	CAN_PACKET_BMS_V_CELL,
	CAN_PACKET_BMS_BAL,
	CAN_PACKET_BMS_TEMPS,
	CAN_PACKET_BMS_HUM,
	CAN_PACKET_BMS_SOC_SOH_TEMP_STAT
} CAN_PACKET_ID; 


typedef struct
{
	int16_t CurSpeed;      
	int16_t ExpSpeed;
	int16_t ActCurrent;
	int16_t OutCurrent;
    int16_t CurPosition;
    int16_t ExpPosition;
    
}vesc;


/*vecs会返回四组数据 以下是四组数据代表的含义*/
typedef struct {
	int id;
	float rpm;
	float current;
	float duty;
} can_status_msg;    

typedef struct {
	int id;
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;

typedef struct {
	int id;
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;

typedef struct {
	int id;
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
} can_status_msg_4;

// Variables

/*
 static  can_status_msg    stat_msgs[CAN_STATUS_MSGS_TO_STORE];
 static can_status_msg_2  stat_msgs_2[CAN_STATUS_MSGS_TO_STORE];
  static can_status_msg_3  stat_msgs_3[CAN_STATUS_MSGS_TO_STORE];
   static can_status_msg_4  stat_msgs_4[CAN_STATUS_MSGS_TO_STORE];
   */






////  CAN 配置
//#define	Motor_CAN               CAN1
//#define Motor_CANSEND           CAN1_Send_Msg
//#define Motor_CANSEND_h         CAN1_Send_Msg




/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) ;
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
void Motor_Analyze_vesc(CanRxMsg *canmsg);
void SetMotor_speed_VESC(uint8_t controller_id,float ExpSpeed);
void comm_can_set_pos(uint8_t controller_id,float ExpPosition);
void comm_can_set_current(uint8_t controller_id,float OutCurrent );
void comm_can_set_current_brake(uint8_t controller_id, float OutCurrent ) ;
