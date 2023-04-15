/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/





#define  ROBOT_DJ_MODULE

#include "vesc.h"
#include "can.h"

vesc     MOTOR_V[15];
char MotorData_v[8];


/*将32位数据转换为4个8位数据 进而发送给vesc
*********************************************************************************************************
*/
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

/*将vesc发送2个8位数据转换为16位数据  
*********************************************************************************************************
*/
static can_status_msg stat_msgs[CAN_STATUS_MSGS_TO_STORE];
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

/*将vesc发送4个8位数据转换为32位数据 
*********************************************************************************************************
*/

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}


/*
*********************************************************************************************************
*                                            	DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
* Brief    : 利用VESC控制电机速度
*Param(s)   controller_id
*           ExpSpeed 期望速度
*
*            
*
* Return(s): none.0
*
*********************************************************************************************************
*/


void SetMotor_speed(uint8_t controller_id,float ExpSpeed) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)ExpSpeed, &send_index);
    while(Motor_CANSEND_Extend (controller_id|(uint16_t)CAN_PACKET_SET_RPM << 8, buffer) == CAN_TxStatus_NoMailBox){};
  
}

/*
*********************************************************************************************************
* Brief    : 利用VESC控制电机位置
*Param(s)   controller_id
*          ExpPosition 期望位置
*
*            
*
* Return(s): none.
*
*********************************************************************************************************
*/
void comm_can_set_pos(uint8_t controller_id,float ExpPosition) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(ExpPosition* 1000000.0), &send_index);
	while(Motor_CANSEND_Extend (controller_id|(uint16_t)CAN_PACKET_SET_POS << 8,buffer) == CAN_TxStatus_NoMailBox){};
}
/*
*********************************************************************************************************
* Brief    : 利用VESC控制电机电流
*Param(s)   controller_id
*          OutCurrent 期望电流
*
*            
*
* Return(s): none.
*
*********************************************************************************************************
*/

void comm_can_set_current(uint8_t controller_id, float OutCurrent ) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(OutCurrent* 1000.0), &send_index);
	while(Motor_CANSEND_Extend (controller_id |(uint16_t)CAN_PACKET_SET_CURRENT << 8,buffer) == CAN_TxStatus_NoMailBox){};
	
}

/*
*********************************************************************************************************
* Brief    : 利用VESC控制电机电流
*Param(s)   controller_id
*          OutCurrent 期望电流
*
*            
*
* Return(s): none.
*
*********************************************************************************************************
*/

void comm_can_set_current_brake(uint8_t controller_id, float OutCurrent ) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(OutCurrent* 1000.0), &send_index);
	while(Motor_CANSEND_Extend (controller_id |(uint16_t)CAN_PACKET_SET_CURRENT_BRAKE << 8,buffer) == CAN_TxStatus_NoMailBox){};
	
}


/**
 * Get status message by id. 通过id获取状态信息
 *
 * @param id
 * Id of the controller that sent the status message. 
 *
 * @return
 * The message or 0 for an invalid id. 
 */
/// @brief 


/*
*********************************************************************************************************
* Brief    : 用于接受并解析电调发送的信息
*
* Param(s) : uint32_t eid  在开始时会接收到一个32位开头的数据 ，这个数据中包含CANID和CAN命令，其中后八位为id 右移八位获得21位CAN命令
             *data8用于存储接受的数据              
*
* Return(s): none.
*
*********************************************************************************************************
*/


void Motor_Analyze_vesc(CanRxMsg *canmsg)
{
   static int i;
   char Motor_Id;
   uint32_t  eid=canmsg->ExtId;
   uint8_t id = eid & 0xFF;//取低8位识别为id
   CAN_PACKET_ID cmd = eid >> 8;//取其高21位（29-8）为指令
  
   for(i = 0; i < canmsg->DLC; i++)
   {MotorData_v[i] = canmsg->Data[i];} 
    
	 switch (cmd) {
       case CAN_PACKET_STATUS: 

    MOTOR_V[id].CurSpeed =  ((uint32_t) MotorData_v[0]) << 24 |
					        ((uint32_t)MotorData_v[1]) << 16 |
					        ((uint32_t) MotorData_v[2]) << 8 |
					        ((uint32_t) MotorData_v[3]);

	MOTOR_V[id].ActCurrent =((uint16_t) MotorData_v[4]) << 8 |
					        ((uint16_t) MotorData_v[5]);
							   break;

    
     case CAN_PACKET_STATUS_4:

	MOTOR_V[id]. CurPosition=((uint16_t) MotorData_v[6]) << 8 |
					        ((uint16_t) MotorData_v[7]);				      

	    break;
		default:
		break;
	 }



}


/*  */
 /*void Motor_Analyze_vesc(uint32_t eid, uint8_t *data8) 
{
	int32_t ind = 0;
	uint8_t id = eid & 0xFF;//取低8位识别为id
	CAN_PACKET_ID cmd = eid >> 8;//取其高21位（29-8）为指令


  switch (cmd) {
	 case CAN_PACKET_STATUS: 
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg *stat_tmp = &stat_msgs[i];
			if (stat_tmp->id == id || stat_tmp->id == -1) {
				ind = 0;
				stat_tmp->id = id;
				stat_tmp->rpm = (float)buffer_get_int32(data8, &ind);
				stat_tmp->current = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp->duty = (float)buffer_get_int16(data8, &ind) / 1000.0;
				MOTOR_V[id].CurSpeed=(int16_t) ( stat_tmp->rpm);
				MOTOR_V[id].ActCurrent=(int16_t)(stat_tmp->current);
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_2:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_2 *stat_tmp_2 = &stat_msgs_2[i];
			if (stat_tmp_2->id == id || stat_tmp_2->id == -1) {
				ind = 0;
				stat_tmp_2->id = id;
				stat_tmp_2->amp_hours = (float)buffer_get_int32(data8, &ind) / 1e4;
				stat_tmp_2->amp_hours_charged = (float)buffer_get_int32(data8, &ind) / 1e4;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_3:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_3 *stat_tmp_3 = &stat_msgs_3[i];
			if (stat_tmp_3->id == id || stat_tmp_3->id == -1) {
				ind = 0;
				stat_tmp_3->id = id;
				stat_tmp_3->watt_hours = (float)buffer_get_int32(data8, &ind) / 1e4;
        
				stat_tmp_3->watt_hours_charged = (float)buffer_get_int32(data8, &ind) / 1e4;
				break;
			}
		}
		break;

	  case CAN_PACKET_STATUS_4:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_4 *stat_tmp_4 = &stat_msgs_4[i];
			if (stat_tmp_4->id == id || stat_tmp_4->id == -1) {
				ind = 0;
				stat_tmp_4->id = id;
				stat_tmp_4->temp_fet = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->temp_motor = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->current_in = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->pid_pos_now = (float)buffer_get_int16(data8, &ind) / 50.0;
				MOTOR_V[id]. CurPosition=(int16_t) (stat_tmp_4->pid_pos_now);
				break;
			}
		}
		break;
	
    }
}*/



