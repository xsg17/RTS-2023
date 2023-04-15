/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  CHASSIS_MODULE
#include "chassis.h"
#include "PID.h"

/*
*********************************************************************************************************
*                                            	DEFINES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

CANOpen_OD Card_1_Init[] = { {0,1,0,0,0}, {1,0x1601,0,0,0}, {1,0x1401,2,0x01,0}, {4,0x1601,1,0x23410020,0}, {1,0x1A01,0,0,0},
							 {1,0x1801,2,0x01,0}, {4,0x1A01,1,0x60690020,0}, {4,0x1A01,2,0x60770010,0}, {1,0x1601,0,1,0}, {1,0x1A01,0,2,0},
							 {4,0x1005,0,0x40000080,0}, {1,0x6060,0,3,0},{2,0x6040,0,0x3F,0}, {0,0,0,0,0}};
//
//CANOpen_OD Card_2_Init[] = { {0,2,0,0,0}, {1,0x1601,0,0,0}, {1,0x1401,2,0x01,0}, {4,0x1601,1,0x23410020,0}, {1,0x1A01,0,0,0},
//							 {1,0x1801,2,0x01,0}, {4,0x1A01,1,0x60690020,0}, {4,0x1A01,2,0x60770010,0}, {1,0x1601,0,1,0},
//							 {1,0x1A01,0,2,0},{1,0x6060,0,3,0},{2,0x6040,0,0x3F,0}, {0,0,0,0,0}};
//
//CANOpen_OD Card_3_Init[] = { {0,3,0,0,0}, {1,0x1601,0,0,0}, {1,0x1401,2,0x01,0}, {4,0x1601,1,0x23410020,0}, {1,0x1A01,0,0,0},
//							 {1,0x1801,2,0x01,0}, {4,0x1A01,1,0x60690020,0}, {4,0x1A01,2,0x60770010,0}, {1,0x1601,0,1,0},
//							 {1,0x1A01,0,2,0},{1,0x6060,0,3,0},{2,0x6040,0,0x3F,0},{0,0,0,0,0}};
//
//CANOpen_OD Card_4_Init[] = { {0,4,0,0,0}, {1,0x1601,0,0,0}, {1,0x1401,2,0x01,0}, {4,0x1601,1,0x23410020,0}, {1,0x1A01,0,0,0},
//							 {1,0x1801,2,0x01,0}, {4,0x1A01,1,0x60690020,0}, {4,0x1A01,2,0x60770010,0}, {1,0x1601,0,1,0}, 
//							 {1,0x1A01,0,2,0},{1,0x6060,0,3,0},{2,0x6040,0,0x3F,0}, {0,0,0,0,0}};

CANOpen_OD *CardInitptr[] = {
	Card_1_Init, 
//	Card_2_Init, 
//	Card_3_Init, 
//	Card_4_Init
};

/*
*********************************************************************************************************
* Brief    : PDO call-back function.Update motor data.
*
* Param(s) : node - CAN address of the motor,correspond to the index of 'Motor_Data'.
*			 msg  - Received CAN message (PDO, include the velocity and torque information of the motor).
*
* Return(s): none.
*
*********************************************************************************************************
*/

void Update_MotorData (uint8_t CANaddress, uint8_t *msg)
{
	Mecanum[CANaddress].Motor_V = (int32_t)(msg[0] | (msg[1] << 8) | (msg[2] << 16) | (msg[3] << 24));
	Mecanum[CANaddress].Motor_Torq = (int16_t)(msg[4] | (msg[5] << 8));
}

/*
*********************************************************************************************************
* Brief    : SYNC message call-back function.Compute all wheels' velocity for the next instant.
*
* Param(s) : 
*
* Return(s): none.
*
*********************************************************************************************************
*/

void PathTrack_SemPost (void)
{
	OS_ERR err; 
	
	OSTaskSemPost(&TaskPathTrackTCB, OS_OPT_POST_NONE, &err);
}
	
void PolyCoeff_Calc (VTrack *desire, PolyBlock *coeff)
{
	float v0,vf,p0,pf;
	float delT,delX,delY;
	
	delX = desire->Vf.Vx - desire->V0.Vx;
	delY = desire->Vf.Vy - desire->V0.Vy;
	delT = sqrt(delX*delX + delY*delY) / 1000.0;
		
	v0 = desire->V0.Vx;
	vf = desire->Vf.Vx;
	p0 = desire->start.x;
	pf = desire->end.x;
	coeff->x.a0 = p0;
	coeff->x.a1 = v0;
	coeff->x.a2 = (3*(pf-p0)/pow(delT,2)) - ((vf+2*v0)/delT);
	coeff->x.a3 = ((vf+v0)/pow(delT,2)) - (2*(pf-p0)/pow(delT,3));
	
	v0 = desire->V0.Vy;
	vf = desire->Vf.Vy;
	p0 = desire->start.y;
	pf = desire->end.y;
	coeff->y.a0 = p0;
	coeff->y.a1 = v0;
	coeff->y.a2 = (3*(pf-p0)/pow(delT,2)) - ((vf+2*v0)/delT);
	coeff->y.a3 = ((vf+v0)/pow(delT,2)) - (2*(pf-p0)/pow(delT,3));
	
	v0 = desire->V0.Vz;
	vf = desire->Vf.Vz;
	p0 = desire->start.z;
	pf = desire->end.z;
	coeff->z.a0 = p0;
	coeff->z.a1 = v0;
	coeff->z.a2 = (3*(pf-p0)/pow(delT,2)) - ((vf+2*v0)/delT);
	coeff->z.a3 = ((vf+v0)/pow(delT,2)) - (2*(pf-p0)/pow(delT,3));	
}

void Next_Velocity (PolyBlock *coeff, Velocity* vNext, Point* pNext, uint32_t k)
{
	float a0,a1,a2,a3;
	float t = k*SYNC_CYCLE/1000.0;
	
	a0 = coeff->x.a0;
	a1 = coeff->x.a1;
	a2 = coeff->x.a2;
	a3 = coeff->x.a3;
	pNext->x = a0 + a1*t + a2*pow(t,2) + a3*pow(t,3);
	vNext->Vx = a1 + 2*a2*t + 3*a3*pow(t,2);
	
	a0 = coeff->y.a0;
	a1 = coeff->y.a1;
	a2 = coeff->y.a2;
	a3 = coeff->y.a3;
	pNext->y = a0 + a1*t + a2*pow(t,2) + a3*pow(t,3);
	vNext->Vy = a1 + 2*a2*t + 3*a3*pow(t,2);
	
	a0 = coeff->z.a0;
	a1 = coeff->z.a1;
	a2 = coeff->z.a2;
	a3 = coeff->z.a3;       
	pNext->x = a0 + a1*t + a2*pow(t,2) + a3*pow(t,3);
	vNext->Vz = a1 + 2*a2*t + 3*a3*pow(t,2);
}

/*
*********************************************************************************************************
* Brief    : Configure a A_card network.(configure each amplfier through SDO).
*
* Param(s) : net - CAN network pointer.
*
* Return(s): none.
*
*********************************************************************************************************
*/

void Chassis_Init (void)
{	
	uint8_t i;
	
	for(i = 0; i < WHEELNUM+1; i++)
	{
		Mecanum[i].Motor_V = 0;
		Mecanum[i].Motor_Torq = 0;
	}
	
	acc_Chassis.NodeNum = 1;
	acc_Chassis.Acc_CANSend = CHASSIS_CANSEND;
	acc_Chassis.InitSeq = CardInitptr;
	acc_Chassis.PDO1hook = Update_MotorData;
	acc_Chassis.SYNChook = PathTrack_SemPost;
	acc_Chassis.SYNC_Node = CAN_SYNCNODE;
	acc_Chassis.SYNC_cycle = SYNC_CYCLE;
	acc_Chassis.SDO_Check = 0;
	Acc_NetWork_Init(&acc_Chassis);

}




