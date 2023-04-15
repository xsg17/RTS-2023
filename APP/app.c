/*
*********************************************************************************************************
*                                              头文件
*********************************************************************************************************
*/

#define   APP_MODULE
#include  <includes.h>

/*
*********************************************************************************************************
*                                            	参数定义
*********************************************************************************************************
*/

#define ACTION_OK       0x01

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

static  OS_TCB   TaskStartTCB;
static  CPU_STK  TaskStartStk[TASK_START_STK_SIZE];

//static  OS_TCB   TaskSpeedCtrl3508TCB;
//static  CPU_STK  TaskSpeedCtrl3508Stk[TASK_SPEEDCTRL3508_STK_SIZE];

static  OS_TCB   TaskSpeedCtrlTCB;
static  CPU_STK  TaskSpeedCtrlStk[TASK_SPEEDCTRL_STK_SIZE];

static  OS_TCB   TaskBTcomTCB;
static  CPU_STK  TaskBTcomStk[TASK_BTCOM_STK_SIZE];
extern  C620  	MOTOR[9];
float Mt = 0;
uint8_t count = 0;

OS_FLAG_GRP InitStatus;


/*
*********************************************************************************************************
*                                           函数原型
*********************************************************************************************************
*/

static  void	TaskStart    		  (void *p_arg);
static  void	TaskSpeedCtrl    	(void *p_arg);
static  void	TaskBTcom         (void *p_arg);

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


int main() // 主函数
{
  OS_ERR err;
  
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

static void TaskStart (void *p_arg)
{
  OS_ERR  err;
  
  (void)p_arg;
  
  CPU_Init(); // Initialize the uC/CPU services
  
  Mem_Init(); // Initialize Memory Managment Module
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
extern u16 motor_test=1;
u32 testspeed=100;
static void TaskSpeedCtrl (void *p_arg)
{
  OS_ERR err;
  CPU_TS ts;
  (void)&p_arg;

  
  OSTimeDlyHMSM(0u, 0u, 0u, 500u, OS_OPT_TIME_HMSM_STRICT, &err);      //2020.4.9 
  

  while (DEF_TRUE) 
  {

    Sendwheel_Vel();//5,6,7,8速度环以及数据发送
    
    OSTimeDlyHMSM(0u, 0u, 0u, 5u, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}

char flag_arbitraryPos = 0;  // 设置位置环
/*
 * 蓝牙无线操作
 * 
 * 
 * 
*/

extern uint16_t CH[18];  // 通道值
extern uint16_t CH_MEM[18];

int speed_temp;
double speed_mapping;
static void TaskBTcom (void *p_arg) 
{
  OS_ERR err;
  CPU_TS ts;
  (void)&p_arg;

 OSTimeDlyHMSM(0u, 0u, 0u, 500u, OS_OPT_TIME_HMSM_STRICT, &err); 
 
  timer_init();
  stepper_motor_init();

  while (1) 
  {    /////////////////底盘移动部分///////////////////////////////
    
    
    if((Pulley1 != Pulley1_MEM)||(abs(Pulley1 - Pulley1_MEM) >=5))      //速度映射
    {
      speed_mapping =  Pulley1 / 150.0;
    }
    
    if(RockerR_Horizontal != RockerR_Horizontal_MEM)                            //右摇杆水平移动->机器人平移
    {
      world.Vx = deadarea_jugde(RockerR_Horizontal,974,150) *speed_mapping;
    }
    
    if(RockerL_Horizontal != RockerL_Horizontal_MEM)                            //左摇杆水平移动->机器人旋转
    {
      world.W = -1*deadarea_jugde(RockerL_Horizontal,990,150)*speed_mapping/20;
    }
    
    if(RockerR_Vertical != RockerR_Vertical_MEM)                            //右摇杆竖直移动->机器人前进
    {
      world.Vy = deadarea_jugde(RockerR_Vertical,987,150)*speed_mapping;
    }
    
    if(Pulley2 != Pulley2_MEM)
    {
      Mt = get_position((Pulley2-172.0)/200.0 + 30.0);
      stepper_motor_run(Mt - current_position);
    } 
//    
//    if(button1 >900)
//    {
//   
//      switch(count){
//      case 0:
//         Pre_capture();
//         if(Pos_flag_2006()&&Pos_flag_3508()) count++;
//         else count=0;
//         break;
//      case 1:
//        Capture();  
//        if(Release_flag()){
//            count++;
//        }
//        else count=1;
//          break;
//       case 2:
//        //OSTimeDlyHMSM(0u, 0u, 0u, 500u, OS_OPT_TIME_HMSM_STRICT, &err);
//          Release_capture();
//          if(Pos_flag_3508()){
//              count=-1;//停下来
//          }
//      }
// 
//       
//    } 
    
    for(int i = 0; i < 18; i++)                     
    {
      CH_MEM[i] = CH[i];
    }

    OSTimeDlyHMSM(0u, 0u, 0u, 5u, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}
  
