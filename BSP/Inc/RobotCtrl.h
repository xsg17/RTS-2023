/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "includes.h"

/*
*********************************************************************************************************
*                                                EXTERNS
*********************************************************************************************************
*/

#ifdef   ROBOT_CTRL_MODULE
#define  ROBOT_CTRL_EXT
#else
#define  ROBOT_CTRL_EXT  extern
#endif

//****************************************************************           ???           ****************************************************************
#ifdef   CHASSIS_MODULE
#define  CHASSIS_EXT
#else
#define  CHASSIS_EXT  extern
#endif
//****************************************************************           ???          ****************************************************************

/*
*********************************************************************************************************
*                                             	 DEFINES
*********************************************************************************************************
*/

// CAN configuration
#define SYNC_CYCLE        5  // 100 ms

// coefficient needed by transform formula
#define THREE_OMNI_D      386.3 // distance between wheel center and spin center
#define FOUR_OMNI_D       280.0
#define FOUR_Mecunum_L    320.0
#define FOUR_Mecunum_l    355.0

// wheel data 
#define WHEELNUM        4
#define	WHEEL_R         100.0 // radius(mm)
#define WHEEL_FORMULA   Formula_4Omni

// motor data 
#define GEAR_RATIO      (1.0/1.0)


#define YES 1
#define NO  0



/*
*********************************************************************************************************
*                                             EXPORTED_TYPES
*********************************************************************************************************
*/

// Speed data structure 
typedef struct
{
  float x; // mm
  float y; // mm
  float z; // degree
  
}Point;

typedef struct   //Ax + BY + C = 0
{
  float A;
  float B;
  float C;
}Line;


typedef struct
{
  float Vx;
  float Vy;
  float W;
  
}Velocity; // unit: mm/s, rad/s


typedef struct
{
  Velocity curSpeed; // (mm/s, mm/s, rad/s)
  Velocity expSpeed;
  Velocity outSpeed;
  Point expPos; // (mm, mm, rad)
  
}MOTION;

typedef struct
{
  float pos_x1; // mm
  float pos_x2; // mm
  float pos_z; // degree
  
}US;

// Position data structure
typedef struct
{
  float pos_x; // mm
  float pos_y; // mm
  float zangle; // degree
  float xangle; // degree
  float yangle; // degree
  float w_z;
  float theta; // rad
  float Vx;
  float Vy;
}POSE;

typedef struct
{
  float pos_x[100]; // mm
  float pos_y[100]; // mm
  float theta[100]; // rad
  float W[100];
  float V[100];
  float Vdirection[100];
}POSE_SET;


// Laser pos
typedef struct
{
  float pos_x; // mm
  float pos_y; // mm
  float pos_z; // degree
  
}LASER;

// Move method
typedef enum
{
  P2P = 0,
  MULTI,
  PCL,
  
}MOVE;

// Control method
typedef enum
{
  MANUAL = 0,
  AUTO,
  
}CTRL;

typedef enum // �����˵�ǰ����λ��
{
  Start = 0,
  LZ1, // ����
  LZ2, // ����
  MeetPoint1,
  MeetPoint2,
  
}BOT;

typedef struct
{
  MOVE RunMode;
  CTRL OP;
  BOT  State;
  
  void (*Move2Point) (uint8_t);
  void (*WaitREADY) (void);
  void (*DONE) (void);
  
}ACT;

typedef struct
{
  char ID;
  int16_t current; // robomodule����������
  int16_t velocity;
  int32_t position;
  int32_t realposition;
  
}BDC;


// Accleration 
typedef struct
{
  float ax;
  float ay;
  float az;
  
}Accelerate;


typedef struct
{
  /* Velocity profile parameter */
  Point qA;               // Start point,     uint: mm
  Point qB;               // End Point,       uint: mm
  Point delAB;			//delAB 			unit:mm
  uint8_t end;
  int32_t Dist;			// 	Distance		uint: mm
  int32_t Acc;
  int32_t Dec;
  int32_t Vlin;          // mm/s, rad/s
  int32_t AccTime;
  int32_t Z_AccTime;
  float 	DecTheta;
  float 	AccTheta;
  float 	Wz;
  float	Z_Acc;
  float	Z_Dec;
  float	Theta;
  float 	DecDist;
}PclData;

typedef struct
{
  int32_t Motor_V;		  // 0.1 enc counts/sec
  int16_t Motor_Torq;		// rated torque / 1000
  
  int32_t Target_V;
  
}Motor_Data;

extern Velocity robo;
extern Velocity world;
extern PclData  PclBlock;

/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/

ROBOT_CTRL_EXT MOTION     RoboMotion;
ROBOT_CTRL_EXT POSE       RobotPos;
ROBOT_CTRL_EXT POSE       RobotExpPos;
ROBOT_CTRL_EXT US         RobotPosUS;
ROBOT_CTRL_EXT LASER      RobotPosition;
ROBOT_CTRL_EXT ACT        RobotCTRL;
ROBOT_CTRL_EXT Point      COULMN_POS[40];
ROBOT_CTRL_EXT Point      COULMN_ForestPOS[20];
ROBOT_CTRL_EXT Point      JCC[15];
ROBOT_CTRL_EXT BDC        BrushDC;
ROBOT_CTRL_EXT CTRL       mode;

ROBOT_CTRL_EXT char      PIDflag;
ROBOT_CTRL_EXT char      FeedbackData;

CHASSIS_EXT Motor_Data Wheel[WHEELNUM+1]; // Index 0 is reserved
CHASSIS_EXT Motor_Data Target_V;

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void Robot_Motion_Init (void);
void Robot_Speed_Update (void);
void Robot_Speed_Ctrl (void);
void SyncSignal (void);

void Pos_Paracal(PclData *profileData,uint8_t col);
void Robot_Pos_Ctrl (void);
void Chassis_Init (void);
void SpeedCtrl (void);
void SendWheel_Vel (void);
void Send_Velocity (Motor_Data *wheel);
void Formula_4Omni(Motor_Data *wheel, Velocity *robo);
void wheelVel_Limit (Motor_Data *wheel);
double deadarea_jugde(float data, int benchmark, int range);
void Robot_Speed_VESC();
void Robot_Speed_C620();
void Robot_Position_Update(double time);
void Navigation(POSE_SET navigation);

/*
*********************************************************************************************************
*                                               MODULE END
*********************************************************************************************************
*/

extern int Speed_Limit; // ����ٶ�
extern char flag_brake;
extern POSE_SET Navigation_Set;