/*
 * @Author: zgates2 2437577662@qq.com
 * @Date: 2023-03-11 16:20:46
 * @LastEditors: zgates2 2437577662@qq.com
 * @LastEditTime: 2023-03-24 20:50:05
 * @FilePath: \�³���2\BSP\Inc\motor.h
 * @Description: ����Ĭ������,������`customMade`, ��koroFileHeader�鿴���� ��������: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
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


    //����������Ŷ���
    #define STEP_PIN    GPIO_Pin_0
    #define DIR_PIN     GPIO_Pin_1

    // ��ʱ������
    #define TIM         TIM2
    #define TIM_RCC     RCC_APB1Periph_TIM2
    #define TIM_IRQ     TIM2_IRQn

// ���������������
#define STEPS_PER_REV   400     // �������ÿת�Ĳ���
#define MAX_SPEED       1000000   // �����������ٶȣ���λΪ����/��  100mm/s
#define ACCEL_RATE      200000   // ����������ٶȣ���λΪ����/��^2
#define TIM_ARR         999  


extern volatile uint16_t step_count;          // ����������������
extern volatile int32_t step_remain;          // �������ʣ�ಽ��
extern volatile float target_position;        // Ŀ��λ�ã���λΪmm
extern volatile float current_position;       // ��ǰλ�ã���λΪmm
extern volatile double  step_speed;           // ���������ǰ�ٶ�
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


