/*
*********************************************************************************************************
*                                               MODULE
*********************************************************************************************************
*/

#ifndef  APP_CFG_MODULE_PRESENT
#define  APP_CFG_MODULE_PRESENT

/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/

#ifdef   APP_MODULE
#define  APP_EXT
#else
#define  APP_EXT  extern
#endif


/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

#define  TASK_START_PRIO                		12u
#define	 TASK_LED_PRIO							8u
#define  TASK_PATHTRACK_PRIO                	3u
#define  TASK_SPEEDCTRL_PRIO                	4u
#define	 TASK_ACT_PRIO							9u
#define	 TASK_BTCOM_PRIO						6u
#define	 TASK_RELAY_PRIO						7u
#define	 TASK_DJ_PRIO                                                   5u
//#define  TASK_SPEEDCTRL3508_PRIO                	                3u
/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*                             Size of the task stacks (# of OS_STK entries)
*********************************************************************************************************
*/

#define	 TASK_LED_STK_SIZE						128u

#define  TASK_START_STK_SIZE           			256u
#define  TASK_PATHTRACK_STK_SIZE           		256u
#define  TASK_SPEEDCTRL_STK_SIZE           		512u
#define	 TASK_ACT_STK_SIZE						256u
#define	 TASK_BTCOM_STK_SIZE					256u
#define	 TASK_RELAY_STK_SIZE					256u
#define	 TASK_DJ_STK_SIZE					    256u

/*
*********************************************************************************************************
*                                     TRACE / DEBUG CONFIGURATION
*********************************************************************************************************
*/

#ifndef  TRACE_LEVEL_OFF
#define  TRACE_LEVEL_OFF                        0u
#endif

#ifndef  TRACE_LEVEL_INFO
#define  TRACE_LEVEL_INFO                       1u
#endif

#ifndef  TRACE_LEVEL_DBG
#define  TRACE_LEVEL_DBG                        2u
#endif

#define  APP_TRACE_LEVEL                        TRACE_LEVEL_OFF
#define  APP_TRACE                              printf

#define  APP_TRACE_INFO(x)               		((APP_TRACE_LEVEL >= TRACE_LEVEL_INFO)  ? (void)(APP_TRACE x) : (void)0)
#define  APP_TRACE_DBG(x)                		((APP_TRACE_LEVEL >= TRACE_LEVEL_DBG)   ? (void)(APP_TRACE x) : (void)0)

#endif

/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/
APP_EXT OS_SEM SyncSem;
APP_EXT OS_SEM RelayReady;

APP_EXT  OS_TCB   TaskPathTrackTCB;
APP_EXT  OS_TCB   TaskBTcomTCB;

APP_EXT float x_try;
APP_EXT float v_try;
APP_EXT float a_try;
APP_EXT float k_try;
APP_EXT u8 LaserFlag;
