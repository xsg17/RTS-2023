/*
 * @Author: zgates2 2437577662@qq.com
 * @Date: 2023-03-11 16:22:48
 * @LastEditors: zgates2 2437577662@qq.com
 * @LastEditTime: 2023-03-30 18:32:54
 * @FilePath: \�³���2\BSP\Src\motor.c
 * @Description: ����Ĭ������,������`customMade`, ��koroFileHeader�鿴���� ��������: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: zgates2 2437577662@qq.com
 * @Date: 2023-03-11 16:22:48
 * @LastEditors: zgates2 2437577662@qq.com
 * @LastEditTime: 2023-03-24 19:13:12
 * @FilePath: \�³���2\BSP\Src\motor.c
 * @Description: ����Ĭ������,������`customMade`, ��koroFileHeader�鿴���� ��������: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
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


/* ���������������
#define STEPS_PER_REV   400     // �������ÿת�Ĳ���
#define MAX_SPEED       40000    // �����������ٶȣ���λΪ����/��  100mm/s
#define ACCEL_RATE      40 0000   // ����������ٶȣ���λΪ����/��^2
#define TIM_ARR         999 */

// ȫ�ֱ�������
volatile uint16_t step_count = 0;          // ����������������
volatile int32_t step_remain = 0;          // �������ʣ�ಽ��
volatile float target_position = 0;        // Ŀ��λ�ã���λΪmm
volatile float current_position = 0;       // ��ǰλ�ã���λΪmm
volatile long float step_speed = 0;           // ���������ǰ�ٶ�
volatile int32_t step_accel = 0;           // ���������ǰ���ٶ� 
int32_t step = 0;
int direction =0;
int pre_count = 0;
int last_count = 0;

// ��ʱ���жϴ�����
void TIM2_IRQHandler(void)
{
 if(TIM_GetITStatus(TIM, TIM_IT_Update)!= RESET) //���TIM2���ж��Ƿ��Ѿ������������򷵻�SET��1��
 {
     // ���²���������������
    if(step_count > 0)
    {
        GPIO_ToggleBits(GPIOA, STEP_PIN);     //��ת��ƽ
        step_count--;
    }
     
      step_speed = fabs(Stepper_Motor_PID(Stepper_Motor));
     
      // ���µ�ǰλ��
       current_position += (direction == 0)?(step_speed/12000.0):(-1.0*step_speed/12000.0); 
       
      step = step_speed*TIM_ARR/1000;     // ���㲽�������ǰ�ٶȺͼ��ٶ�
      step_remain -= step;
      step_count = step;                 // ���㲽�������ǰ��������������
     
       // ���²����������
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
      // �����ʱ���жϱ�־
     TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
 }
  
}


// ���������ʼ������
void stepper_motor_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // ���ò����������
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Pin = STEP_PIN | DIR_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_ResetBits(GPIOA, STEP_PIN | DIR_PIN);
}


// ��ʱ����ʼ������
void timer_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // ���ö�ʱ��ʱ��
    RCC_APB1PeriphClockCmd(TIM_RCC, ENABLE);

    // ���ö�ʱ����������
    TIM_TimeBaseInitStruct.TIM_Period = 999;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 83;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM, &TIM_TimeBaseInitStruct);

    // ���ö�ʱ���ж�
    NVIC_InitStruct.NVIC_IRQChannel = TIM_IRQ;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_ITConfig(TIM, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM, ENABLE);
}



// ����������ƺ���
void stepper_motor_run(float distance)
{
    // ����Ŀ��λ��
    target_position = current_position + distance;

    // ���㲽�������Ҫת���Ĳ���
     

    // ���ò������ʣ�ಽ��
    step_remain = distance * STEPS_PER_REV / 5;
    //step_remain = distance;

    // �ȴ������������Ŀ��λ��
   // while(step_remain > 0);
}

// ת��λ���븩���ǵĹ�ϵ �� = /0.1099a + 30.7888
float get_position(float angle)
{

   float position = (angle - 30)/0.1099;

    return position;
}

