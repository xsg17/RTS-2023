/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "includes.h"

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

//数据类型转换参数 ：将int16 型数据( AC_V )转换为两个 int8 型数据 ( AC_VH, AC_VL )
extern int AC_Vx, AC_Vy, AC_V;
extern int C1,C2,C3,C4;
//extern int8_t AC_VH, AC_VL;
extern int8_t AC_VH1, AC_VL1,AC_VH2, AC_VL2;

// 无线 PID 调参参数
extern unsigned char PID_data1[15]; // 存储上位机发送的 PID 数据
extern float PID_data2[3]; // 存储处理后的 PID 数据 
//extern unsigned char K_data1[16];
//extern float K_data2[3];
//extern unsigned char MAX_MIN1[11];
//extern float MAX_MIN2[2];
//无线调参参数
extern unsigned char BT_Flag; // 无线调参标志位
extern unsigned char num; // 无线调参长度参数
extern unsigned char Move_Flag; // 行进参数


/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void Data_Change(void); // 数据类型转换 发送给上位机模拟示波器画图使用
void DataToMatlab(void); // 上传给　MATLAB　使用
void UsartTest(void); // USART3 与 USART6 的测试函数
void DataJudge(void); // 无线调参
void WirelessParaAdj(u8 WirelessParametric); // 无线调参接收数据
void K_Judge(void);
void M_Judge(void);
/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/