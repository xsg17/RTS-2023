/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  ROBOTCTRL_MODULE
#include "RobotCtrl.h"
#include "vesc.h"

MOTION   RoboMotion;
MOTION   RoboMotionVESC;
MOTION   RoboMotionC620;
POSE     RobotPos;
POSE     RobotExpPos;
ACT      RobotCTRL;
BDC      BrushDC;

/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/


extern vesc     MOTOR_V[15];
extern C620  	MOTOR[9];

Point error = {0,0,0};
Point target = {0,1000,0};
Point present = {0,0,0};

int Center_Wheel_Length;

Motor_Data wheel[WHEELNUM + 1];
//******************************************************************************
int Speed_Limit = 3000; // 最大速度    3500，超调20cm，未加配重
                                  //   3500，超调11cm,加配重
                                  //   4000，超调24cm，加配重
                                  //   4500，超调55cm，加配重
float Theta_Limit = 5000;
//******************************************************************************

#ifndef PI
#define PI 3.14159265358979f
#endif
// Unit transform constant
const float CONT = (GEAR_RATIO*60) / (2*PI*WHEEL_R);  // [wheel](mm/s) -> [motor](rpm)

// Motor limitation 
const int32_t MOTOR_DEAD_VAL = 0;
const int32_t MOTOR_MAX_SPEED = 8000.0*2000*10/60;         // rpm -> 0.1 enc counts/s

// Three  omini calculation constant
float SIN60d;  // sin(pi/3)
float COS60d;  // cos(pi/3)
float COS45d;

// verioty constants
const double a=10;
const double b=10;

/*
********************************************************************************************************
*                                          	   	VARIABLES
*********************************************************************************************************
*/

char PIDflag = 0;
char FeedbackData = 0;

Velocity robo;
Velocity world;
//PclData  PclBlock;
char ad_flag = 0;
char flag_brake = 0;
//function pointer points to the formula that transforms robot velocity to wheel frame
void (*Xform_Robo2wheel) (Motor_Data*, Velocity*);  

/*
*********************************************************************************************************
*                                          	 PRIVATE FUNCTIONS
*********************************************************************************************************
*/



/*
*********************************************************************************************************
* Brief    : Configure a A_card network.
*			       (initialize stm32 CAN peripheral and configure each amplfier through SDO).
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
  
  SIN60d = sin(PI/3.0);  // sin(pi/3)
  COS60d = cos(PI/3.0);  // cos(pi/3)
  COS45d = cos(PI/4.0);
  
 Xform_Robo2wheel = WHEEL_FORMULA;
  
  for(i = 0; i < WHEELNUM + 1; i++)
  {
    wheel[i].Motor_V = 0;
    wheel[i].Motor_Torq = 0;
    wheel[i].Target_V = 0;
  }
  
  //使用编码器控制电机时才有用
  //RoboDrive_Init();
}



/*
*********************************************************************************************************
* Brief    : Initialize motion & control data structure.
*
* Param(s) :
*
* Return(s): none.
*
*********************************************************************************************************
*/
void Robot_Motion_Init (void)
{
  RoboMotion.expSpeed.Vx = RoboMotion.expSpeed.Vy = RoboMotion.expSpeed.W = 0;
  RoboMotion.expPos.x = RoboMotion.expPos.y = RoboMotion.expPos.z = 0;
  
  RobotCTRL.OP = MANUAL;
  RobotCTRL.State=Start;
}


/*
*********************************************************************************************************
* Brief    : World frame to robot frame transform formula.
*
* Param(s) : world - velocity vector(Vx,Vy,W) in inertia frame. (mm/s, rad/s)
*			 robo  - velocity vector(Vx,Vy,W) in robot frame. (mm/s, rad/s)
*
* Return(s): none.
*
*********************************************************************************************************
*/
void Formula_World2Robo (Velocity *world, Velocity *robo)  //可能有用
{
  float sinTheta = sin(RobotPos.theta);
  float cosTheta = cos(RobotPos.theta);
  
  robo->Vy = world->Vx * cosTheta + world->Vy * sinTheta;
  robo->Vx = -1*(world->Vx * ( -sinTheta ) + world->Vy * cosTheta);
  robo->W = world->W;
}



/*
*********************************************************************************************************
* Brief    : robot frame to wheel frame transform formula (4 omini wheels chassis).
*
* Param(s) : wheel - velocity vector(Vx,Vy,W) in wheel frame. (0.1 enc counts/s)
*			 robo  - velocity vector(Vx,Vy,W) in robot frame. (mm/s, rad/s)
*
* Return(s): none.
*
*********************************************************************************************************
*/
//void Formula_4Omni (Motor_Data *wheel, Velocity *robo)
//{
  //wheel[1].Target_V = CONT * ( -COS45d * robo->Vx + COS45d * robo->Vy - FOUR_OMNI_D * robo->W ); //左上轮1
  //wheel[2].Target_V = CONT * ( -COS45d * robo->Vx - COS45d * robo->Vy - FOUR_OMNI_D * robo->W ); //右下轮2
  //wheel[3].Target_V = CONT * (  COS45d * robo->Vx - COS45d * robo->Vy - FOUR_OMNI_D * robo->W ); //右上轮3
  //wheel[4].Target_V = CONT * (  COS45d * robo->Vx + COS45d * robo->Vy - FOUR_OMNI_D * robo->W ); //左下轮4
//}

/**
 * @brief 求麦轮四个轮子的速度
 * 
 * @param Vx 本体坐标系中的水平速度
 * @param Vy 本体坐标系中的竖直速度
 * @param W 角速度
 */
void Formula_4Omni(Motor_Data *wheel, Velocity *robo)
{
	//v为轮子转速，vx，vy为本体坐标系中的速度，a,b为位置矢量在x，y轴上的分量，w为角速度。
	wheel[1].Target_V=   robo->Vy + robo->Vx - robo->W*(a+b);
	wheel[2].Target_V=  -robo->Vy + robo->Vx - robo->W*(a+b);
	wheel[3].Target_V=  -robo->Vy - robo->Vx - robo->W*(a+b);
	wheel[4].Target_V=   robo->Vy - robo->Vx - robo->W*(a+b);

  
}


/*
*********************************************************************************************************
* Brief    : Pack all the functions needed to send new velocity value to all motors.
*
* Param(s) :
*
* Return(s): none.
*
*********************************************************************************************************
*/
void Sendwheel_Vel (void)
{
  //世界坐标系向机器人坐标系转换
  Formula_World2Robo (&world, &robo);
  //计算四个轮子的期望速度
  Formula_4Omni(&wheel[0], &robo);
  //对轮子速度进行约束
  wheelVel_Limit(&wheel[0]);
  Send_Velocity(&wheel[0]);
  
  //将轮子的期望速度转换为期望电流
  Motor_Speed_Ctrl_C620();  //2021.4.9为消除冲突，修改名称Motor_Speed_Ctrl->Motor_Speed_Ctrl_C620

  //向C620电调发送期望电流值
  // SetMotor();
  // SetMotor_h();
}




/*
*********************************************************************************************************
* Brief    : 
*
* Param(s) : none
*
* Return(s): none
*
*********************************************************************************************************
*/
void Robot_Pos_Ctrl (void)
{
//  static float recVx = 0,recVy = 0;
//  float jiasudu_x,jiasudu_y;
  RoboMotion.expSpeed.Vx = PIDCal_pos(&posxPID, RobotExpPos.pos_x - RobotPos.pos_x);
  RoboMotion.expSpeed.Vy = PIDCal_pos(&posyPID, RobotExpPos.pos_y - RobotPos.pos_y);
//  if(RoboMotion.expSpeed.Vx < 0 || RoboMotion.expSpeed.Vy < 0)
//  {
//  if(RoboMotion.expSpeed.Vx > recVx || RoboMotion.expSpeed.Vy > recVy)
//  {
//    jiasudu_x = 1000*(RoboMotion.expSpeed.Vx - recVx)/SYNC_CYCLE;
//    jiasudu_y = 1000*(RoboMotion.expSpeed.Vy - recVy)/SYNC_CYCLE; 
//    
//    if(jiasudu_x < 15000)
//    {
//      RoboMotion.expSpeed.Vx = recVx + 15000*SYNC_CYCLE/1000;
//    }
//    if(jiasudu_y < 15000)
//    {
//      RoboMotion.expSpeed.Vy = recVy + 15000*SYNC_CYCLE/1000;
//    }
//   }
//  }

  // 速度限幅
  if(RoboMotion.expSpeed.Vx > Speed_Limit)
  {
    RoboMotion.expSpeed.Vx = Speed_Limit;
  } 
  else if(RoboMotion.expSpeed.Vx < -Speed_Limit)
  {
    RoboMotion.expSpeed.Vx = -Speed_Limit;
  }

  if(RoboMotion.expSpeed.Vy > Speed_Limit)
  {
    RoboMotion.expSpeed.Vy = Speed_Limit;
  }
  else if(RoboMotion.expSpeed.Vy < -Speed_Limit)
  {
    RoboMotion.expSpeed.Vy = -Speed_Limit;
  }
  
  if(RoboMotion.expSpeed.W > Theta_Limit)
  {
    RoboMotion.expSpeed.W = Theta_Limit;
  }
  else if(RoboMotion.expSpeed.W < -Theta_Limit)
  {
    RoboMotion.expSpeed.W = -Theta_Limit;
  }

//  recVx = RoboMotion.expSpeed.Vx;
//  recVy = RoboMotion.expSpeed.Vy;
}



/*
*********************************************************************************************************
* Brief    : PID speed controller (in robot frame).
*
* Param(s) :
*
* Return(s): none.
*
*********************************************************************************************************
*/
//float observe = 0;
//float ob_tmpy = 0;
int os = 0;
void Robot_Speed_Ctrl (void)                           //分段式PID控制
{
  RoboMotion.outSpeed.Vx = -RoboMotion.expSpeed.Vx;
  RoboMotion.outSpeed.Vy = -RoboMotion.expSpeed.Vy;
  RoboMotion.outSpeed.W = 0;  // 更新一版，每次算完都清零，近似位置式pid算法，可以达成目标，速度较为震动，参数可调
  
  if( abs(RobotPos.theta) <= PI*10/180.0)     //角度PID——小误差（误差范围在15°以内）
  {
    RoboMotion.outSpeed.W += PIDCal( &ThetaPID2, ( RoboMotion.expPos.z - RobotPos.theta ) ) * 10 ;
  }
  else                                       //大误差
  {
    RoboMotion.outSpeed.W += PIDCal( &ThetaPID1, ( RoboMotion.expPos.z - RobotPos.theta ) ) * 15 ; // 8
  }
  
  if( abs(RoboMotion.expSpeed.Vx) <= 200 )     //x小
  {
    RoboMotion.outSpeed.Vx -= PIDCal( &VelxPID2, RoboMotion.expSpeed.Vx - RoboMotion.curSpeed.Vx );
  }
  else                                          //x大
  {
    RoboMotion.outSpeed.Vx -= PIDCal( &VelxPID1, RoboMotion.expSpeed.Vx - RoboMotion.curSpeed.Vx );
    if( abs(RoboMotion.expSpeed.Vx) <= abs(RoboMotion.curSpeed.Vx) )
    {
     // RoboMotion.outSpeed.Vx = RoboMotion.outSpeed.Vx * 1.2;
    }
  }
  
  
  if( abs(RoboMotion.expSpeed.Vy) <= 200 )     //y小
  {
    RoboMotion.outSpeed.Vy -= PIDCal( &VelyPID2, RoboMotion.expSpeed.Vy - RoboMotion.curSpeed.Vy );
  }
  else                                        //y大
  {
    RoboMotion.outSpeed.Vy -= PIDCal( &VelyPID1, RoboMotion.expSpeed.Vy - RoboMotion.curSpeed.Vy );
    if( abs(RoboMotion.expSpeed.Vy) <= abs(RoboMotion.curSpeed.Vy) )
    {
    //  RoboMotion.outSpeed.Vy = RoboMotion.outSpeed.Vy * 1.2;
    }
  }
  
  if( flag_brake == 1 )
      RoboMotion.outSpeed.Vx =  RoboMotion.outSpeed.Vy = RoboMotion.outSpeed.W = 0; // 急停
  
  if( RoboMotion.outSpeed.Vx >= 15000 )
  {
    RoboMotion.outSpeed.Vx = 15000;
  }
  if( RoboMotion.outSpeed.Vx <= -15000 )
  {
    RoboMotion.outSpeed.Vx = -15000;
  }
  
  if( RoboMotion.outSpeed.Vy >= 15000 )
  {
    RoboMotion.outSpeed.Vy = 15000;
  }
  if( RoboMotion.outSpeed.Vy <= -15000 )
  {
    RoboMotion.outSpeed.Vy = -15000;
  }
  
   if( RoboMotion.outSpeed.W >= 3000 )
  {
    RoboMotion.outSpeed.W = 3000;
  }
  if( RoboMotion.outSpeed.W <= -3000 )
  {
    RoboMotion.outSpeed.W = -3000;
  }
  
}





/*
*********************************************************************************************************
* Brief    : SYNC message call-back function.Calculate robot speed(x,y,z) and
*            inform path task to calculate next velocity.
*
* Param(s) :
*
* Return(s): none.
*
*********************************************************************************************************
*/
void SyncSignal(void)
{
  OS_ERR  err;
  
  OSSemPost( &SyncSem, OS_OPT_POST_ALL, &err );
}



/* 
*********************************************************************************************************
* Brief    : Use the data from encoders to calculate robot's speed in x,y,z direction.
*               Vx - mm/s
*               Vy - mm/s
*               Wz - rad/s
*
* Param(s) :
*
* Return(s): none.
*
*********************************************************************************************************
*/
unsigned char flag_puzzle = 0;
void Robot_Speed_Update (void)
{
  static float recx = 0,recy = 0,recz = 0;
  static float recVx = 0,recVy = 0,recW = 0;
  
  RoboMotion.curSpeed.Vx = 1000*(RobotPos.pos_x - recx)/SYNC_CYCLE;
  RoboMotion.curSpeed.Vy = 1000*(RobotPos.pos_y - recy)/SYNC_CYCLE;
  RoboMotion.curSpeed.W =  1000*(RobotPos.theta - recz)/SYNC_CYCLE;
  
  recx = RobotPos.pos_x;
  recy = RobotPos.pos_y;
  recz = RobotPos.theta;
  recVx = RoboMotion.curSpeed.Vx;
  recVy = RoboMotion.curSpeed.Vy;
  recW = RoboMotion.curSpeed.W;
}


void Robot_Speed_C620()
{
  float tempX, tempY, tempW;
  tempX = (MOTOR[1].CurSpeed-MOTOR[2].CurSpeed)/2.0*187.0/3591.0;  //*0.377
  
  tempY = (MOTOR[1].CurSpeed+MOTOR[2].CurSpeed-MOTOR[3].CurSpeed-MOTOR[4].CurSpeed)/4.0*187.0/3591.0;  //*0.050
  
  tempW= (MOTOR[1].CurSpeed+MOTOR[2].CurSpeed+MOTOR[3].CurSpeed+MOTOR[4].CurSpeed)/4.0*187.0/3591.0;  //*0.385
  
  RoboMotionC620.curSpeed.Vy = ((0.3 * fabs(tempY) >= fabs(tempX)) || (0.3 * fabs(tempW) >= fabs(tempX))) ? 0.0 : (tempX*PI*120.0/60.0);
  RoboMotionC620.curSpeed.Vx = ((0.3 * fabs(tempX) >= fabs(tempY)) || (0.3 * fabs(tempW) >= fabs(tempY))) ? 0.0 : (tempY*PI*120.0/60.0);
  RoboMotionC620.curSpeed.W  = ((0.3 * fabs(tempX) >= fabs(tempW)) || (0.3 * fabs(tempY) >= fabs(tempW))) ? 0.0 : (tempW*PI*120.0/200.0/60.0);
       
}

void Robot_Position_Update(float time)
{
  present.y += (RoboMotionC620.curSpeed.Vy * 0.04);
  present.z += (RoboMotionC620.curSpeed.W * 0.04);
  present.x += (RoboMotionC620.curSpeed.Vx * 0.04);
}






/*
*********************************************************************************************************
                                以下函数未被调用
*********************************************************************************************************
*/



/*
*********************************************************************************************************
* Brief    : Limit motor output speed.
*
* Param(s) :
*
* Return(s): none.
*
*********************************************************************************************************
*/
void wheelVel_Limit (Motor_Data *wheel)
{
  int i;
  
  for(i = 1; i <= WHEELNUM; i++)
  {
    if((wheel[i].Target_V < 0) && (wheel[i].Target_V >= -MOTOR_DEAD_VAL))
      wheel[i].Target_V = 0;
    
    else if((wheel[i].Target_V > 0) && (wheel[i].Target_V <= MOTOR_DEAD_VAL))
      wheel[i].Target_V = 0;
    
    else if((wheel[i].Target_V < 0) && (wheel[i].Target_V < -MOTOR_MAX_SPEED))
      wheel[i].Target_V = -MOTOR_MAX_SPEED;
    
    else if((wheel[i].Target_V > 0) && (wheel[i].Target_V > MOTOR_MAX_SPEED))
      wheel[i].Target_V = MOTOR_MAX_SPEED;
  }
}



/*
*********************************************************************************************************
* Brief    : Path tracking coefficient calculation
*
* Return(s): none.
*
*********************************************************************************************************
*/
/*******************************************************************************
函 数 名：Pos_Paracal(PclData *profileData,uint8_t col)
描    述：计算x、y、z方向的加减速时间、距离及最大速度。
*******************************************************************************/
void Pos_Paracal( PclData *profileData, uint8_t col )
{
  float acc_dist,dec_dist;
  float acc_theta,dec_theta;
  
  profileData->qA.x = RobotPos.pos_x;
  profileData->qA.y = RobotPos.pos_y;
  profileData->qA.z = RobotPos.theta;   //当前位置
  
  profileData->qB.x = COULMN_POS[col].x;
  profileData->qB.y = COULMN_POS[col].y;
  profileData->qB.z = COULMN_POS[col].z;  //目标位置
  
  profileData->delAB.x = profileData->qB.x - profileData->qA.x;
  profileData->delAB.y = profileData->qB.y - profileData->qA.y;
  profileData->delAB.z = profileData->qB.z - profileData->qA.z;      // 两者位置差距
  
  profileData->Dist  = sqrt( pow( profileData->delAB.x, 2 ) + pow( profileData->delAB.y, 2 ) );  //直线差距
  profileData->Theta = fabs( profileData->delAB.z );  //角度差距
  if(ad_flag ==0)
  {
    profileData->Acc  = 15000;  // 加速加速度//15000
    profileData->Dec  = 2100;   // 减速加速度//2100
    profileData->Vlin = 4000;   // 最大速度//4000
    //		acc_dist = (pow(profileData->Vlin,2)-pow(RoboMotion.curSpeed.Vx,2))/profileData->Acc/2.0;  // 最大加速距离
  }
  if( ad_flag == 1 )
  {
    profileData->Acc  = 4000;  // 加速加速度//4000
    profileData->Dec  = 1250;   // 减速加速度//1250
    profileData->Vlin = 4000;   // 最大速度//4000
  }
  if( ad_flag == 2 )
  {
    profileData->Acc  = 4500;  // 加速加速度//4000
    profileData->Dec  = 1250;   // 减速加速度//1250
    profileData->Vlin = 4500;   // 最大速度//4500
  }
  
  if( ad_flag != 0 )
  { acc_dist = ( pow( profileData->Vlin, 2 ) - pow( RoboMotion.curSpeed.Vx, 2 ) ) / profileData->Acc / 2.0; }  // 最大加速距离
  dec_dist = pow( profileData->Vlin, 2 ) / profileData->Dec / 2.0;  // 最大减速距离
  if( profileData->Dist < ( acc_dist + dec_dist ) )   //距离小RoboMotion.curSpeed.Vx于最大加速距离最大减速距离和
  {
    float vlin_square = ( profileData->Dist ) * 2.0 * profileData->Acc * profileData->Dec / 1.0 / ( profileData->Acc + profileData->Dec ); //最大速度平方
    if( vlin_square < 100 * 100 )
    {
      profileData->AccTime = 0;  //加速时间
      profileData->DecDist = 0;  //减速距离
      profileData->Vlin = 100;   //最大速度
    }
    else
    {
      profileData->Vlin = sqrt( vlin_square );// 最大速度
      profileData->AccTime = profileData->Vlin * 1000.0 / profileData->Acc;//加速时间
      profileData->DecDist = pow( profileData->Vlin, 2 ) / 2.0 / profileData->Dec;//减速距离
    }
  }
  else
  {
    profileData->AccTime = profileData->Vlin * 1000.0 / profileData->Acc;  //加速时间
    profileData->DecDist = dec_dist;  //减速距离
  }
  
  profileData->Wz    = 0.3;
  profileData->Z_Acc = 0.3;
  profileData->Z_Dec = 0.3;   //z方向 最大速度、加速加速度、减速加速度
  
  acc_theta = pow( profileData->Wz, 2 ) / profileData->Z_Acc /2.0;//加速距离
  dec_theta = pow( profileData->Wz, 2 ) / profileData->Z_Dec /2.0;//减速距离
  
  if( profileData->Theta < ( acc_theta + dec_theta ) )
  {
    float wz_square = ( profileData->Theta ) * 2.0 * profileData->Z_Acc * profileData->Z_Dec / 1.0 / ( profileData->Z_Acc + profileData->Z_Dec );
    if( wz_square < 0.2 * 0.2 )
    {
      profileData->Z_AccTime = 0;
      profileData->DecTheta = 0;
      profileData->Wz = 0.2;
    }
    else
    {
      profileData->Wz = sqrt( wz_square );
      profileData->Z_AccTime = (int32_t)( profileData->Wz * 1000.0 / profileData->Z_Acc );
      profileData->DecTheta = pow( profileData->Wz, 2 ) /2.0 / profileData->Z_Dec;
    }
  }
  else
  {
    profileData->Z_AccTime = (int32_t)( profileData->Wz * 1000.0 / profileData->Z_Acc );
    profileData->DecTheta = dec_theta;
  }
}


/*
*********************************************************************************************************
* Brief    : Send each motor(wheel) velocity through PDO.
*
* Param(s) : wheel - motor speed array.
*
* Return(s): none.
*
*********************************************************************************************************
*/
void Send_Velocity (Motor_Data *wheel)
{
  uint8_t i;
  
  for(i = 0 ;i < WHEELNUM; i++ )
  {
    MOTOR[i+1].ExpSpeed=wheel[i+1].Target_V;
  }
}


/*
*********************************************************************************************************
* Brief    : judge whether the data is in dead area
*
* Param(s) : data-data from rocker.       benchmark-the number when rocker is in center.
*            range- the range of dead area.
*
* Return(s): int.
*
*********************************************************************************************************
*/

double deadarea_jugde(float data, int benchmark, int range)
{
  if (abs(benchmark - data) <= range)
  {
    return 0;
  }
  else if((data > 1700)||(data < 200)) 
  {
    if(data < 200)
    {
      return (200-benchmark);
    }
    else
    {
      return (1700-benchmark);
    }
  }
  else
  {
    return (double)(data-benchmark);
  }
}

