/*
*********************************************************************************************************
*                                              头文件
*********************************************************************************************************
*/

#define APP_MODULE
#include <includes.h>

/*
*********************************************************************************************************
*                                            	参数定义
*********************************************************************************************************
*/

#define ACTION_OK 0x01

/*
*********************************************************************************************************
*                                      	      本地参数
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             任务变量
*********************************************************************************************************
*/

OS_ERR err;

static OS_TCB TaskStartTCB;
static CPU_STK TaskStartStk[TASK_START_STK_SIZE];

// static  OS_TCB   TaskSpeedCtrl3508TCB;
// static  CPU_STK  TaskSpeedCtrl3508Stk[TASK_SPEEDCTRL3508_STK_SIZE];

static OS_TCB TaskSpeedCtrlTCB;
static CPU_STK TaskSpeedCtrlStk[TASK_SPEEDCTRL_STK_SIZE];

static OS_TCB TaskBTcomTCB;
static CPU_STK TaskBTcomStk[TASK_BTCOM_STK_SIZE];
extern C620 MOTOR[9];
float Mt = 0;
uint8_t count = 0;

OS_FLAG_GRP InitStatus;

/*
*********************************************************************************************************
*                                           函数原型
*********************************************************************************************************
*/

static void TaskStart(void *p_arg);
static void TaskSpeedCtrl(void *p_arg);
static void TaskBTcom(void *p_arg);

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/

extern PIDType X, Y, W;


//gxs做尝试

int main() // 主函数
{
  

  CPU_IntDis(); // Disable all Interrupts.

  OSInit(&err); // Init uC/OS-III.

  OSTaskCreate(&TaskStartTCB, // Create the start task
               "Task Start",
               TaskStart,
               0u,
               TASK_START_PRIO,
               &TaskStartStk[0u],
               (TASK_START_STK_SIZE / 10u),
               TASK_START_STK_SIZE,
               0u,
               0u,
               0u,
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               &err);

  OSStart(&err); // Start multitasking (i.e. give control to uC/OS-III).

  while (DEF_ON)
  {
    // Should Never Get Here.
  }
}

/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static void TaskStart(void *p_arg)
{
  OS_ERR err;

  (void)p_arg;

  CPU_Init(); // Initialize the uC/CPU services

  Mem_Init();  // Initialize Memory Managment Module
  Math_Init(); // Initialize Mathematical Module

  CPU_IntEn();
  BSP_Init(); // Initialize BSP functions

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err); // Compute CPU capacity with no task running
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();
#endif

  OSFlagCreate(&InitStatus,
               "Init Status",
               (OS_FLAGS)0,
               &err);

  OSSemCreate(&SyncSem,
              "Sync Semaphore",
              0,
              &err);

  OSSemCreate(&RelayReady,
              "Relay Ready",
              0,
              &err);

  OSTaskCreate(&TaskSpeedCtrlTCB,
               "Task SpeedCtrl",
               TaskSpeedCtrl,
               0u,
               TASK_SPEEDCTRL_PRIO,
               &TaskSpeedCtrlStk[0u],
               (TASK_SPEEDCTRL_STK_SIZE / 10u),
               TASK_SPEEDCTRL_STK_SIZE,
               0u,
               0u,
               0u,
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               &err);

  OSTaskCreate(&TaskBTcomTCB,
               "BT Com",
               TaskBTcom,
               0u,
               TASK_BTCOM_PRIO,
               &TaskBTcomStk[0u],
               (TASK_BTCOM_STK_SIZE / 10u),
               TASK_BTCOM_STK_SIZE,
               0u,
               0u,
               0u,
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               &err);

  INFO("TaskStart OK\n");

  while (DEF_TRUE)
  {
    // Task body, always written as an infinite loop.
    OSTimeDlyHMSM(0u, 0u, 0u, 300u, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}

/*
 * 速度控制
 *
 *
 *
 */
extern u16 motor_test = 1;
u32 testspeed = 100;
static void TaskSpeedCtrl(void *p_arg)
{
  OS_ERR err;
  CPU_TS ts;
  (void)&p_arg;

  OSTimeDlyHMSM(0u, 0u, 0u, 500u, OS_OPT_TIME_HMSM_STRICT, &err); // 2020.4.9

  while (DEF_TRUE)
  {

    Sendwheel_Vel(); // 5,6,7,8速度环以及数据发送
    Motor_Speed_Ctrl_C610();
    Motor_Position_Ctrl_C610();
    SetMotor_C610();
    Motor_Speed_Ctrl_C620();
    Motor_Position_Ctrl();
    // 向C620电调发送期望电流值
    SetMotor();
    SetMotor_h();

    OSTimeDlyHMSM(0u, 0u, 0u, 5u, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}

char flag_arbitraryPos = 0; // 设置位置环
/*
 * 蓝牙无线操作
 *
 *
 *
 */

extern uint16_t CH[18]; // 通道值
extern uint16_t CH_MEM[18];
double preTick = 0;
double Tick = 0;
int speed_temp;
double speed_mapping;
int trigger_flag = 1, capture_flag = 1, release_flag = 1, switch_flag = 0;
static void TaskBTcom(void *p_arg)
{
  OS_ERR err;
  CPU_TS ts;
  (void)&p_arg;

  OSTimeDlyHMSM(0u, 0u, 0u, 500u, OS_OPT_TIME_HMSM_STRICT, &err);
  Action_Init();
  timer_init();
  stepper_motor_init();
  //Pre_capture();
  while (1)
  { /////////////////底盘移动部分///////////////////////////////
//
//    if ((LS != LS_MEM) || (abs(LS - LS_MEM) >= 5)) // 速度映射
//    {
//      speed_mapping = (LS - 353.0) / 1343.0 * 3.0;
//    }
//
//    if (RockerR_Horizontal != RockerR_Horizontal_MEM) // 右摇杆水平移动->机器人平移
//    {
//      world.Vy = deadarea_jugde(RockerR_Horizontal, 1024, 150) * speed_mapping;
//    }
//
//    if (RockerL_Horizontal != RockerL_Horizontal_MEM) // 左摇杆水平移动->机器人旋转
//    {
//      world.W = deadarea_jugde(RockerL_Horizontal, 1024, 150) * speed_mapping / 180;
//    }
//
//    if (RockerR_Vertical != RockerR_Vertical_MEM) // 右摇杆竖直移动->机器人前进
//    {
//      world.Vx = deadarea_jugde(RockerR_Vertical, 1024, 150) * speed_mapping;
//    }
//    if(RD != RD_MEM){
//      SetMotor_speed_VESC(109,(RD - 353)/(1695-353)*50);
//    }
//    if(LD != LD_MEM){
//      SetMotor_speed_VESC(127,(LD - 353)/(1695-353)*50);
//    }
//    if (SH > 1000)
//    {
//      if (trigger_flag)
//      {
//        MOTOR[7].Expvel = 6000;
//        MOTOR[7].Exparg += 157293.5; // 一圈
//        trigger_flag = 0;
//      }
//    }
//    else
//    {
//      trigger_flag = 1;
//    }
//    if ((SF >= (SF_MEM + 400)) || (SF <= (SF_MEM - 400)))
//    {
//      if (capture_flag && (!switch_flag))
//      {
//        Capture();
//        capture_flag = 0;
//        release_flag = 1;
//        switch_flag = !switch_flag; 
//      }
//      else if(release_flag && switch_flag)
//      {
//          Re_capture();
//          release_flag = 0;
//          capture_flag = 1;
//          switch_flag = !switch_flag;
//      }
//    }
//    else
//    {
//      capture_flag = 1;
//      release_flag = 1;
//    }
//    
//    if(RS != RS_MEM)
//    {
//      Mt = get_position((RS-200.0)/200.0 + 31.0);
//      stepper_motor_run(Mt - current_position);
//    }
    
      Navigation(Navigation_Set);
    
//    if(SB != SB_MEM)
//    {
//      if (SB >= 1200){
//        SetMotor_speed_VESC(1,0);
//        SetMotor_speed_VESC(2,0);
//      }
//      else if (SB <= 500){
//        SetMotor_speed_VESC(1,500);
//        SetMotor_speed_VESC(2,500);
//      }
//      else{
//        SetMotor_speed_VESC(1,200);
//        SetMotor_speed_VESC(2,200);
//      }
//    }
//    for (int i = 0; i < 18; i++)
//    {
//      CH_MEM[i] = CH[i];
//    }
  
//    if(switch_flag == 0){
//      OSTimeDlyHMSM(0u, 0u, 13u, 0u, OS_OPT_TIME_HMSM_STRICT, &err);
//      switch_flag++;
//    } 
//    Robot_Position_Update(0);
//    Tick = OSTimeGet(&err);
//    Robot_Speed_C620();
//    Robot_Position_Update((Tick-preTick)/18300.0);
    //Robot_Position_Update(TIM_GetCounter(TIM5)/10000.0);
//    Position_PID(X, Y, W);
//    Robot_Pos_Ctrl();
//    Robot_Speed_Ctrl();
//    preTick = Tick;
//    TIM_SetCounter(TIM5, 0);
    OSTimeDlyHMSM(0u, 0u, 0u, 5u, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}
