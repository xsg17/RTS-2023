/*
 * @Author: zgates2 2437577662@qq.com
 * @Date: 2023-03-11 16:22:48
 * @LastEditors: zgates2 2437577662@qq.com
 * @LastEditTime: 2023-03-30 18:32:54
 * @FilePath: \新程序2\BSP\Src\motor.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: zgates2 2437577662@qq.com
 * @Date: 2023-03-11 16:22:48
 * @LastEditors: zgates2 2437577662@qq.com
 * @LastEditTime: 2023-03-24 19:13:12
 * @FilePath: \新程序2\BSP\Src\motor.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  MOTOR_MODULE
#include "motor.h"
#include "PID.h"

/*
*********************************************************************************************************
*                                            	ANOTHER
*********************************************************************************************************
*/


/* 步进电机参数定义
#define STEPS_PER_REV   400     // 步进电机每转的步数
#define MAX_SPEED       40000    // 步进电机最大速度，单位为步数/秒  100mm/s
#define ACCEL_RATE      40 0000   // 步进电机加速度，单位为步数/秒^2
#define TIM_ARR         999 */

// 全局变量定义
volatile uint16_t step_count = 0;          // 步进电机脉冲计数器
volatile int32_t step_remain = 0;          // 步进电机剩余步数
volatile float target_position = 0;        // 目标位置，单位为mm
volatile float current_position = 0;       // 当前位置，单位为mm
volatile long float step_speed = 0;           // 步进电机当前速度
volatile int32_t step_accel = 0;           // 步进电机当前加速度 
int32_t step = 0;
int direction =0;
int pre_count = 0;
int last_count = 0;

// 定时器中断处理函数
void TIM2_IRQHandler(void)
{
 if(TIM_GetITStatus(TIM, TIM_IT_Update)!= RESET) //检查TIM2的中断是否已经发生，发生则返回SET（1）
 {
     // 更新步进电机脉冲计数器
    if(step_count > 0)
    {
        GPIO_ToggleBits(GPIOA, STEP_PIN);     //反转电平
        step_count--;
    }
     
      step_speed = fabs(Stepper_Motor_PID(Stepper_Motor));
     
      // 更新当前位置
       current_position += (direction == 0)?(step_speed/12000.0):(-1.0*step_speed/12000.0); 
       
      step = step_speed*TIM_ARR/1000;     // 计算步进电机当前速度和加速度
      step_remain -= step;
      step_count = step;                 // 计算步进电机当前步距和脉冲计数器
     
       // 更新步进电机方向
     if((target_position - current_position)< -0.05)
        {
            GPIO_ResetBits(GPIOA, DIR_PIN);
            direction = 1; 
            step_remain = -1*step_remain;
        }
        else if((target_position - current_position) > 0.05)
        {
            GPIO_SetBits(GPIOA, DIR_PIN);
            direction = 0; 
        }
     else
     {     
     }
      //GPIO_ToggleBits(GPIOA, DIR_PIN);
      // 清除定时器中断标志
     TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
 }
  
}


// 步进电机初始化函数
void stepper_motor_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // 配置步进电机引脚
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Pin = STEP_PIN | DIR_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_ResetBits(GPIOA, STEP_PIN | DIR_PIN);
}


// 定时器初始化函数
void timer_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 配置定时器时钟
    RCC_APB1PeriphClockCmd(TIM_RCC, ENABLE);

    // 配置定时器基本参数
    TIM_TimeBaseInitStruct.TIM_Period = 999;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 83;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM, &TIM_TimeBaseInitStruct);

    // 配置定时器中断
    NVIC_InitStruct.NVIC_IRQChannel = TIM_IRQ;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_ITConfig(TIM, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM, ENABLE);
}



// 步进电机控制函数
void stepper_motor_run(float distance)
{
    // 计算目标位置
    target_position = current_position + distance;

    // 计算步进电机需要转动的步数
     

    // 设置步进电机剩余步数
    step_remain = distance * STEPS_PER_REV / 5;
    //step_remain = distance;

    // 等待步进电机到达目标位置
   // while(step_remain > 0);
}

// 转动位置与俯仰角的关系 β = /0.1099a + 30.7888
float get_position(float angle)
{

   float position = (angle - 30)/0.1099;

    return position;
}

