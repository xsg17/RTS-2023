#include "remote.h"
#include "BT.h"
#include "tim.h"

void TIM3_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 配置定时器时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // 配置定时器基本参数
    TIM_TimeBaseInitStruct.TIM_Period = 999;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 83;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

    // 配置定时器中断
    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}


    
void TIM3_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM3, TIM_IT_Update)!= RESET) //检查TIM3的中断是否已经发生，发生则返回SET（1）
    {
      if((Pulley1 != Pulley1_MEM)||(abs(Pulley1 - Pulley1_MEM) >=5))      //速度映射
    {
      speed_mapping =  Pulley1 / 150.0;
    }
    
    if(RockerR_Horizontal != RockerR_Horizontal_MEM)                            //右摇杆水平移动->机器人平移
    {
      world.Vx = deadarea_jugde(RockerR_Horizontal,974,150) *speed_mapping;
      OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    }
    
    if(RockerL_Horizontal != RockerL_Horizontal_MEM)                            //左摇杆水平移动->机器人旋转
    {
      world.W = -1*deadarea_jugde(RockerL_Horizontal,990,150)*speed_mapping/20;
      OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    }
    
    if(RockerR_Vertical != RockerR_Vertical_MEM)                            //右摇杆竖直移动->机器人前进
    {
      world.Vy = deadarea_jugde(RockerR_Vertical,987,150)*speed_mapping;
      OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    }
    
    if(Pulley2 != Pulley2_MEM)
    {
      Mt = get_position((Pulley2-200)/200 + 30);
      stepper_motor_run(100 - current_position);
      OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    } 

    
    for(int i = 0; i < 18; i++)                     
    {
      CH_MEM[i] = CH[i];
    }
   }
   TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}