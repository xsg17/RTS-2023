/*
 * @Author: zgates2 2437577662@qq.com
 * @Date: 2023-03-11 16:20:46
 * @LastEditors: zgates2 2437577662@qq.com
 * @LastEditTime: 2023-03-24 20:50:05
 * @FilePath: \新程序2\BSP\Inc\motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "includes.h"

/*
*********************************************************************************************************
*                                              DEFINE
*********************************************************************************************************
*/


    //步进电机引脚定义
    #define STEP_PIN    GPIO_Pin_0
    #define DIR_PIN     GPIO_Pin_1

    // 定时器定义
    #define TIM         TIM2
    #define TIM_RCC     RCC_APB1Periph_TIM2
    #define TIM_IRQ     TIM2_IRQn

// 步进电机参数定义
#define STEPS_PER_REV   400     // 步进电机每转的步数
#define MAX_SPEED       1000000   // 步进电机最大速度，单位为步数/秒  100mm/s
#define ACCEL_RATE      200000   // 步进电机加速度，单位为步数/秒^2
#define TIM_ARR         999  


extern volatile uint16_t step_count;          // 步进电机脉冲计数器
extern volatile int32_t step_remain;          // 步进电机剩余步数
extern volatile float target_position;        // 目标位置，单位为mm
extern volatile float current_position;       // 当前位置，单位为mm
extern volatile double  step_speed;           // 步进电机当前速度
extern volatile int32_t step_accel; 

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void timer_init(void);
void stepper_motor_init(void);
void stepper_motor_run(float distance);
float get_position(float angle);


