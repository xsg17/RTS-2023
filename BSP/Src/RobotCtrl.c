/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  ROBOTCTRL_MODULE
#include "RobotCtrl.h"
#include "vesc.h"
#include "BT.h"

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



POSE_SET Navigation_Set ={
{0.298,1.184,2.646,4.672,7.25,10.368,14.014,18.176,22.842,28,33.638,39.744,46.306,53.312,60.75,68.608,76.874,85.536,94.582,104,113.778,123.904,134.366,145.152,156.25,167.648,179.334,191.296,203.522,216,228.718,241.664,254.826,268.192,281.75,295.488,309.394,323.456,337.662,352,366.458,381.024,395.686,410.432,425.25,440.128,455.054,470.016,485.002,500,514.998,529.984,544.946,559.872,574.75,589.568,604.314,618.976,633.542,648,662.338,676.544,690.606,704.512,718.25,731.808,745.174,758.336,771.282,784,796.478,808.704,820.666,832.352,843.75,854.848,865.634,876.096,886.222,896,905.418,914.464,923.126,931.392,939.25,946.688,953.694,960.256,966.362,972,977.158,981.824,985.986,989.632,992.75,995.328,997.354,998.816,999.702,1000},
{0.0297,0.1176,0.2619,0.4608,0.7125,1.0152,1.3671,1.7664,2.2113,2.7,3.2307,3.8016,4.4109,5.0568,5.7375,6.4512,7.1961,7.9704,8.7723,9.6,10.4517,11.3256,12.2199,13.1328,14.0625,15.0072,15.9651,16.9344,17.9133,18.9,19.8927,20.8896,21.8889,22.8888,23.8875,24.8832,25.8741,26.8584,27.8343,28.8,29.7537,30.6936,31.6179,32.5248,33.4125,34.2792,35.1231,35.9424,36.7353,37.5,38.2347,38.9376,39.6069,40.2408,40.8375,41.3952,41.9121,42.3864,42.8163,43.2,43.5357,43.8216,44.0559,44.2368,44.3625,44.4312,44.4411,44.3904,44.2773,44.1,43.8567,43.5456,43.1649,42.7128,42.1875,41.5872,40.9101,40.1544,39.3183,38.4,37.3977,36.3096,35.1339,33.8688,32.5125,31.0632,29.5191,27.8784,26.1393,24.3,22.3587,20.3136,18.1629,15.9048,13.5375,11.0592,8.4681,5.7624,2.9403,0},
{1.55E-05,0.000121924,0.000405259,0.000945956,0.001819178,0.003094886,0.004837949,0.007108263,0.009960858,0.013446017,0.017609382,0.022492074,0.028130804,0.034557982,0.041801835,0.049886521,0.058832235,0.06865533,0.079368426,0.090980523,0.103497116,0.116920307,0.131248917,0.146478602,0.162601964,0.179608662,0.197485532,0.216216691,0.235783659,0.256165465,0.277338764,0.299277948,0.321955262,0.345340915,0.36940319,0.394108566,0.419421821,0.445306151,0.471723284,0.498633586,0.525996183,0.55376907,0.581909221,0.610372708,0.63911481,0.668090129,0.697252698,0.726556103,0.755953585,0.785398163,0.814842741,0.844240224,0.873543628,0.902706198,0.931681517,0.960423619,0.988887106,1.017027257,1.044800143,1.072162741,1.099073043,1.125490175,1.151374506,1.176687761,1.201393136,1.225455412,1.248841064,1.271518379,1.293457563,1.314630862,1.335012668,1.354579635,1.373310795,1.391187665,1.408194363,1.424317725,1.43954741,1.45387602,1.467299211,1.479815804,1.491427901,1.502140996,1.511964092,1.520909806,1.528994491,1.536238345,1.542665523,1.548304253,1.553186945,1.55735031,1.560835468,1.563688064,1.565958377,1.567701441,1.568977148,1.569850371,1.570391068,1.570674403,1.570780854,1.570796327},
{0.004618612,0.018103114,0.039904981,0.069487003,0.106323276,0.149899209,0.199711516,0.255268226,0.316088674,0.381703507,0.451654681,0.52549546,0.60279042,0.683115446,0.766057734,0.851215786,0.938199418,1.026629754,1.116139226,1.206371579,1.296981866,1.387636449,1.478013001,1.567800504,1.656699251,1.744420843,1.830688192,1.915235519,1.997808355,2.07816354,2.156069226,2.231304872,2.303661249,2.372940436,2.438955822,2.501532106,2.560505298,2.615722716,2.667042988,2.714336053,2.757483157,2.79637686,2.830921027,2.861030837,2.886632775,2.907664638,2.924075533,2.935825875,2.942887389,2.945243113,2.942887389,2.935825875,2.924075533,2.907664638,2.886632775,2.861030837,2.830921027,2.79637686,2.757483157,2.714336053,2.667042988,2.615722716,2.560505298,2.501532106,2.438955822,2.372940436,2.303661249,2.231304872,2.156069226,2.07816354,1.997808355,1.915235519,1.830688192,1.744420843,1.656699251,1.567800504,1.478013001,1.387636449,1.296981866,1.206371579,1.116139226,1.026629754,0.938199418,0.851215786,0.766057734,0.683115446,0.60279042,0.52549546,0.451654681,0.381703507,0.316088674,0.255268226,0.199711516,0.149899209,0.106323276,0.069487003,0.039904981,0.018103114,0.004618612,0},
{1.295878413,5.149927636,11.46226008,20.06927545,30.74790083,43.22137221,57.16640755,72.22158546,87.99671231,104.0829352,120.0633383,135.5237485,150.0634695,163.3056672,174.9071358,184.5671937,192.0354756,197.1184217,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,199.6842945,197.1184217,192.0354756,184.5671937,174.9071358,163.3056672,150.0634695,135.5237485,120.0633383,104.0829352,87.99671231,72.22158546,57.16640755,43.22137221,30.74790083,20.06927545,11.46226008,5.149927636,1.295878413,0},
{0.099494949,0.098979592,0.098453608,0.097916667,0.097368421,0.096808511,0.096236559,0.095652174,0.095054945,0.094444444,0.093820225,0.093181818,0.092528736,0.091860465,0.091176471,0.09047619,0.089759036,0.08902439,0.088271605,0.0875,0.086708861,0.085897436,0.085064935,0.084210526,0.083333333,0.082432432,0.081506849,0.080555556,0.079577465,0.078571429,0.077536232,0.076470588,0.075373134,0.074242424,0.073076923,0.071875,0.070634921,0.069354839,0.068032787,0.066666667,0.065254237,0.063793103,0.062280702,0.060714286,0.059090909,0.057407407,0.055660377,0.053846154,0.051960784,0.05,0.047959184,0.045833333,0.043617021,0.041304348,0.038888889,0.036363636,0.03372093,0.030952381,0.02804878,0.025,0.021794872,0.018421053,0.014864865,0.011111111,0.007142857,0.002941176,         -0.001515152,-0.00625,-0.011290323,-0.016666667,-0.022413793,-0.028571429,-0.035185185,-0.042307692,-0.05,-0.058333333,-0.067391304,-0.077272727,-0.088095238,-0.1,-0.113157895,-0.127777778,-0.144117647,-0.1625,-0.183333333,-0.207142857,-0.234615385,-0.266666667,-0.304545455,-0.35,-0.405555556,-0.475,-0.564285714,-0.683333333,-0.85,-1.1,-1.516666667,-2.35,-4.85,-10.0}
};

extern vesc     MOTOR_V[15];
extern C620  	MOTOR[9];

Point error   = {0,0,0};
Point target  = {1000,0,0};
Point present = {0,0,0};
Point Reference = {0, 0, 0};
Line  expect  = {0, 0, 0};
Line  speed   = {0, 0, 0}; 

int Center_Wheel_Length;
int Navigation_Point_Order = 0;
int Expect_Line_Create_Flag = 0;
int Speed_Line_Create_Flag = 0;
float Point_Precision = 50;
float Line_Precision = 50;


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
const double a=195;
const double b=195;


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
  
 //Xform_Robo2wheel = WHEEL_FORMULA;
  
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
void Formula_World2Robo (Velocity *world, Velocity *robo)
{
  float sinTheta = 0;
  float cosTheta = 1;
  if(SD > 1000){
    sinTheta = sin(RobotPos.theta);
    cosTheta = cos(RobotPos.theta);
  }
  
  robo->Vx = world->Vx * cosTheta + world->Vy * sinTheta;
  robo->Vy = world->Vx * ( -sinTheta ) + world->Vy * cosTheta;
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
	wheel[1].Target_V=  -robo->Vy - robo->Vx + robo->W*(a+b);
	wheel[2].Target_V=   robo->Vy - robo->Vx + robo->W*(a+b);
	wheel[3].Target_V=   robo->Vy + robo->Vx + robo->W*(a+b);
	wheel[4].Target_V=  -robo->Vy + robo->Vx + robo->W*(a+b);

  
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
//    if(abs(target.x - present.x)<10)
//  {
//    target.x = present.x;
//  }
//   if(abs(target.y - present.y)<10)
//  {
//    target.y = present.y;
//  }

  RoboMotion.expSpeed.Vx = PIDCal_pos(&posxPID, target.x - present.x);
  RoboMotion.expSpeed.Vy = PIDCal_pos(&posyPID, target.y - present.y);
  
 

 
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
  
//  if(RoboMotion.expSpeed.Vz > Theta_Limit)
//  {
//    RoboMotion.expSpeed.Vz = Theta_Limit;
//  }
//  else if(RoboMotion.expSpeed.Vz < -Theta_Limit)
//  {
//    RoboMotion.expSpeed.Vz = -Theta_Limit;
//  }
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
void Robot_Speed_Ctrl (void)                           //分段式PID控制
{
  world.Vy = RoboMotion.expSpeed.Vy;
  world.Vx = RoboMotion.expSpeed.Vx;
  world.W  = RoboMotion.expSpeed.W;  // 更新一版，每次算完都清零，近似位置式pid算法，可以达成目标，速度较为震动，参数可调

 if( abs(RobotPos.theta) <= PI*10/180.0)     //角度PID——小误差（误差范围在15°以内）
 {
   world.W += PIDCal( &ThetaPID2, ( target.z - RobotPos.theta ) ) * 10 ;
 }
 else                                       //大误差
 {
   world.W += PIDCal( &ThetaPID1, ( target.z - RobotPos.theta ) ) * 15 ; // 8
 }
 
 if( abs(RoboMotion.expSpeed.Vx) <= 200 )     //x小
 {
   world.Vx = PIDCal( &VelxPID2, target.x - RoboMotionC620.curSpeed.Vx );
 }
 else                                          //x大
 {
   world.Vx = PIDCal( &VelxPID1, target.x - RoboMotionC620.curSpeed.Vx );
 }
 
 
 if( abs(RoboMotion.expSpeed.Vy) <= 200 )     //y小
 {
   world.Vy = PIDCal( &VelyPID2, target.y - RoboMotionC620.curSpeed.Vy );
 }
 else                                        //y大
 {
   world.Vy = PIDCal( &VelyPID1, target.y - RoboMotionC620.curSpeed.Vy );
 }

 if( flag_brake == 1 )
     world.Vy =  world.Vx = world.W = 0; // 急停
  
  if( world.Vy >= 200 )
  {
    world.Vy = 200;
  }
  if( world.Vy <= -200 )
  {
    world.Vy = -200;
  }
  
  if( world.Vx >= 200 )
  {
    world.Vx = 200;
  }
  if( world.Vx <= -200)
  {
    world.Vx = -200;
  }
  
   if( world.W >= 10 )
  {
    world.W = 10;
  }
  if( world.W <= -10 )
  {
    world.W = -10;
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
//  static float recx = 0,recy = 0,recz = 0;
//  static float recVx = 0,recVy = 0,recW = 0;
  
//  RoboMotion.curSpeed.Vx = 1000*(RobotPos.pos_x - recx)/SYNC_CYCLE;
//  RoboMotion.curSpeed.Vy = 1000*(RobotPos.pos_y - recy)/SYNC_CYCLE;
//  RoboMotion.curSpeed.W =  1000*(RobotPos.theta - recz)/SYNC_CYCLE;
//  
//  recx = RobotPos.pos_x;
//  recy = RobotPos.pos_y;
//  recz = RobotPos.theta;
//  recVx = RoboMotion.curSpeed.Vx;
//  recVy = RoboMotion.curSpeed.Vy;
//  recW = RoboMotion.curSpeed.W;
}



/* 
*********************************************************************************************************
* Brief    : Use the data from encoders to calculate robot's speed in x,y,z direction.
*               Vx - mm/s
*               Vy - mm/s
*               W  - rad/s
*
* Param(s) : none.
*
* Return(s): none.
*
*********************************************************************************************************
*/
void Robot_Speed_C620()
{
  float tempX, tempY, tempW;
//  tempX = (MOTOR[1].CurSpeed-MOTOR[2].CurSpeed)/2.0*187.0/3591.0;  //*0.377
//  
//  tempY = (MOTOR[1].CurSpeed+MOTOR[2].CurSpeed-MOTOR[3].CurSpeed-MOTOR[4].CurSpeed)/4.0*187.0/3591.0;  //*0.050
//  
//  tempW= (MOTOR[1].CurSpeed+MOTOR[2].CurSpeed+MOTOR[3].CurSpeed+MOTOR[4].CurSpeed)/4.0*187.0/3591.0;  //*0.385
//  
//  RoboMotionC620.curSpeed.Vy = ((0.3 * fabs(tempY) >= fabs(tempX)) || (0.3 * fabs(tempW) >= fabs(tempX))) ? 0.0 : (tempX*PI*120.0/60.0);
//  RoboMotionC620.curSpeed.Vx = ((0.3 * fabs(tempX) >= fabs(tempY)) || (0.3 * fabs(tempW) >= fabs(tempY))) ? 0.0 : (tempY*PI*120.0/60.0);
//  RoboMotionC620.curSpeed.W  = ((0.3 * fabs(tempX) >= fabs(tempW)) || (0.3 * fabs(tempY) >= fabs(tempW))) ? 0.0 : (tempW*PI*120.0/200.0/60.0);

  tempX = (MOTOR[1].CurSpeed-MOTOR[2].CurSpeed)/127.0;
  
  tempY = (MOTOR[1].CurSpeed+MOTOR[2].CurSpeed-MOTOR[3].CurSpeed-MOTOR[4].CurSpeed)/254.0;  
  
  tempW= (MOTOR[1].CurSpeed+MOTOR[2].CurSpeed+MOTOR[3].CurSpeed+MOTOR[4].CurSpeed)/254.0;
  
  RoboMotionC620.curSpeed.Vy = ((0.2 * fabs(tempY) >= fabs(tempX)) || (0.2 * fabs(tempW) >= fabs(tempX))) ? 0.0 : (tempX*PI*127.0);

  RoboMotionC620.curSpeed.Vx = ((0.2 * fabs(tempX) >= fabs(tempY)) || (0.2 * fabs(tempW) >= fabs(tempY))) ? 0.0 : (tempY*PI*127.0);

  RoboMotionC620.curSpeed.W  = ((0.2 * fabs(tempX) >= fabs(tempW)) || (0.2 * fabs(tempY) >= fabs(tempW))) ? 0.0 : (tempW*PI*127.0/195.0);
}


/* 
*********************************************************************************************************
* Brief    : Use the robot velocity to calculate robot's position in x,y,z direction.
*               x - mm
*               y - mm
*               z - rad
*
* Param(s) :
*
* Return(s): none.
*
*********************************************************************************************************
*/
void Robot_Position_Update(double time)
{
  present.y = -RobotPos.pos_x;
  present.z = -RobotPos.theta;
  present.x = -RobotPos.pos_y;
  
  Reference.y += RoboMotionC620.curSpeed.Vy*time;
  Reference.z += RoboMotionC620.curSpeed.W*time;
  Reference.x += RoboMotionC620.curSpeed.Vx*time;
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


/*
*********************************************************************************************************
* Brief    : Judgement is 2 point is near
*
* Param(s) :  point1, point2 and range to judge far or near
*
* Return(s): yes(1) or NO(0)
*
*********************************************************************************************************
*/
int Point_is_Near(Point A, Point B, float range){
  if(sqrt(pow((A.x - B.x), 2) + pow((A.y - B.y), 2) <= range)){         //判断两点间距离 (Δx^2 +Δy^2)^-1
    return YES;
  }
  else{
    return NO;
  }
  return NO;
}


/*
*********************************************************************************************************
* Brief    : Judgement is line and a point is near
*
* Param(s) :  line 1, point 1 and range to judge far or near
*
* Return(s): yes(1) or NO(0)
*
*********************************************************************************************************
*/

int Line_Point_is_Near(Line l, Point p, float range){                               //                |Ax + By +c|
  if((abs( l.A*p.x + l.B*p.y + l.C )/(sqrt( pow((l.A), 2) + pow((l.B), 2)))) <= range){     //判断点线距离     -------------
    return YES;                                                                   //                  (A^2 + B^2)^-1          
  }
  else{
    return NO;
  }
}


/*
*********************************************************************************************************
* Brief    : Navigation controls
*
* Param(s) : Waypoints(x, y, w, v, theta) set
*
* Return(s): None
*
*********************************************************************************************************
*/

void Navigation(POSE_SET navigation){
  Point wayPoint = {navigation.pos_x[Navigation_Point_Order], navigation.pos_y[Navigation_Point_Order], navigation.theta[Navigation_Point_Order]};
  float Present_Speed_Line_Distance = 0;
  float CompensationVelocity = 0;
  if(Point_is_Near(present,wayPoint, Point_Precision) == YES){                                     //判断机器人是否抵达路径点
    Navigation_Point_Order++;                                                                      //将当前路径点更换为下一个
    Speed_Line_Create_Flag = NO;
  }
  else{
    
    if(Navigation_Point_Order !=  Expect_Line_Create_Flag){                                             //若现目标点与前目标点间未建立直线方程
      expect.A = 1/(wayPoint.x - navigation.pos_x[Navigation_Point_Order - 1]);                  //  A = 1/(X2 - X1)
      expect.B = 1/(navigation.pos_y[Navigation_Point_Order - 1] - wayPoint.y);                  //  B = 1/(Y1 - Y2)
      expect.C = navigation.pos_y[Navigation_Point_Order - 1]/(wayPoint.y - navigation.pos_y[Navigation_Point_Order - 1]) + // C = Y1/(Y2 - Y1) + X1/(X1 - X2)
                 navigation.pos_x[Navigation_Point_Order - 1]/(navigation.pos_x[Navigation_Point_Order - 1] - wayPoint.x);
      Expect_Line_Create_Flag++;                                                                       //说明现目标点与前目标点间已建立直线方程
    }
    
    if(Line_Point_is_Near(expect, present, Line_Precision) == YES){                                   //若机器人在期望路线上运动，则保持当前目标点期望速度
      CompensationVelocity = 0;
    }
    
    else{                                                                                             //若机器人不在期望路线上运动
      
      if(Speed_Line_Create_Flag != YES){                                                              // 若没有配置速度线 
//        speed.A =  1/(navigation[Navigation_Point_Order+1].Vx - navigation[Navigation_Point_Order-1].Vx);
//        speed.B = -1/(navigation[Navigation_Point_Order-1].Vy - navigation[Navigation_Point_Order+1].Vy);
//        speed.C =     navigation[Navigation_Point_Order-1].pos_y/(navigation[Navigation_Point_Order+1].Vy-navigation[Navigation_Point_Order-1].Vy)- 
//                      navigation[Navigation_Point_Order-1].pos_x/(navigation[Navigation_Point_Order-1].Vx-navigation[Navigation_Point_Order+1].Vx);
        
        speed.A = navigation.Vdirection[Navigation_Point_Order];
        speed.B = -1;
        speed.C = navigation.pos_y[Navigation_Point_Order] -navigation.Vdirection[Navigation_Point_Order]*navigation.pos_x[Navigation_Point_Order];
        
        Speed_Line_Create_Flag = YES;
      }
      
      Present_Speed_Line_Distance = abs(speed.A*present.x + speed.B*present.y + speed.C)/(sqrt(pow((speed.A), 2) + pow((speed.B), 2))); //当前位置与速度线间的误差
      
      CompensationVelocity = PIDCal(&NavigationPID, Present_Speed_Line_Distance);              //补偿速度，与速度线垂直
    }
    world.Vx =   navigation.V[Navigation_Point_Order]/(PI * 127) * 60 * cos(present.z - navigation.Vdirection[Navigation_Point_Order])
               - cos(present.z - tan(-(speed.A/speed.B)) + PI/2) * CompensationVelocity;
    
    world.Vy = navigation.V[Navigation_Point_Order]/(PI * 127) * 60 * sin(present.z - navigation.Vdirection[Navigation_Point_Order])
               + sin(present.z - tan(-(speed.A/speed.B)) + PI/2) * CompensationVelocity;
    
    world.W  = PIDCal(&NavigationWPID, navigation.W[Navigation_Point_Order] - RobotPos.w_z) + PIDCal(&NavigationTPID, navigation.theta[Navigation_Point_Order] - RobotPos.theta);
  } 
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
  
//  profileData->qA.x = RobotPos.pos_x;
//  profileData->qA.y = RobotPos.pos_y;
//  profileData->qA.z = RobotPos.theta;   //当前位置
  
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