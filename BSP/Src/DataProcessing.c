/*
*********************************************************************************************************
*                                                 头文件
*********************************************************************************************************
*/

#define  DATAPROCESSING_MODULE
#include "DataProcessing.h"

/*
************************************************************************************************************
*                                            上位机通信数据处理
************************************************************************************************************
*/

//数据类型转换参数 ：将int16 型数据( AC_V )转换为两个 int8 型数据 ( AC_VH, AC_VL )
int AC_Vx, AC_Vy, AC_V,AC_SP,AC_SV,AC_D;
int8_t AC_VH1, AC_VL1,AC_VH2, AC_VL2,AC_SPH,AC_SPL,AC_SVH,AC_SVL,AC_DL,AC_DH;
int C1,C2,C3,C4;
void Data_Change(void) // 数据类型转换 发送给上位机模拟示波器画图使用
{
//    AC_Vx = RoboMotion.expSpeed.Vx; // 实际速度Vx
   /* AC_Vy = RoboMotion.expSpeed.Vy; // 
    AC_SP = RobotPos.pos_y;
    AC_SV = RoboMotion.curSpeed.Vy;
    AC_D  =  100*RobotPos.zangle;*/
    C1=current[1];
    C2=current[2];
    C3=current[3];
    C4=current[4];
//    AC_V = Move_V; // 合成后的速度V float -> int
//  AC_V = RoboMotion.curSpeed.Vy;
// int16 型数据转换为两个 int8 型数据
  /*AC_VH1 = AC_Vx >> 8;
  AC_VL1 = AC_Vx & 0x0ff;*/

// int16 型数据转换为两个 int8 型数据
  /*AC_VH2 = AC_Vy >> 8;
  AC_VL2 = AC_Vy & 0x0ff;
  AC_SPH = AC_SP >> 8;
  AC_SPL = AC_SP & 0x0ff;
  AC_SVH = AC_SV >> 8;
  AC_SVL = AC_SV & 0x0ff;
  AC_DH = AC_D >> 8;
  AC_DL = AC_D & 0x0ff;*/
AC_VH2 = C1 >> 8;
  AC_VL2 = C1 & 0x0ff;
  AC_SPH = C2 >> 8;
  AC_SPL = C2& 0x0ff;
  AC_SVH =C3 >> 8;
  AC_SVL = C3& 0x0ff;
  AC_DH = C4 >> 8;
  AC_DL =C4 & 0x0ff;
//  SendToDraw2( AC_SPL, AC_SPH, AC_VL2, AC_VH2,AC_SVL, AC_SVH,AC_DL,AC_DH); // USART 6发送给上位机
}

void DataToMatlab(void) // 上传给　MATLAB　使用 不推荐使用
{
// 使用 Matlab 时使用以下两句指令给上位机
//  send_char(MotorData_send[2][2]);
//  send_char(MotorData_send[2][3]);
}

//void UsartTest(void) // USART3 与 USART6 的测试函数
//{
//  // 通信端口测试
//  {
//    // USART3
////     send_uchar( 0x33 );
////     send_uchar( 0xBD );
//    // USART6
//    send_uchar_draw( 0x66 );
//    send_uchar_draw( 0xAB );
//  }
//}

unsigned char PID_data1[15]; // 存储上位机发送的 PID 数据
//unsigned char K_data1[16];
//unsigned char MAX_MIN1[11];
float PID_data2[3]; // 存储处理后的 PID 数据
//float K_data2[3];
//float MAX_MIN2[2];
// 数据发送格式 ：（ PID参数 ）FP....I....D.... , ( 速度方向参数 ) A ：前进 ， B：后退
void DataJudge(void) // 无线 PID 调参 赋值
{
  int a,b,c,d; // 中间变量
  
  a = PID_data1[1] - 0x30; // char 合成为 int
  b = PID_data1[2] - 0x30; // char 合成为 int
  c = PID_data1[3] - 0x30; // char 合成为 int
  d = PID_data1[4] - 0x30; // char 合成为 int
  PID_data2[0] = a * 1 + b*0.1 + c * 0.01 + d * 0.001; // P = a * 10 + b + c * 0.1 + d * 0.01
  a = PID_data1[6] - 0x30; // char 合成为 int
  b = PID_data1[7] - 0x30; // char 合成为 int
  c = PID_data1[8] - 0x30; // char 合成为 int
  d = PID_data1[9] - 0x30; // char 合成为 int
  PID_data2[1] = a * 1 + b*0.1 + c * 0.01 + d * 0.001; // I = a * 10 + b + c * 0.1 + d * 0.01
  a = PID_data1[11] - 0x30; // char 合成为 int
  b = PID_data1[12] - 0x30; // char 合成为 int
  c = PID_data1[13] - 0x30; // char 合成为 int
  d = PID_data1[14] - 0x30; // char 合成为 int
  PID_data2[2] = a * 1 + b*0.1 + c * 0.01 + d * 0.001; // D = a * 10 + b + c * 0.1 + d * 0.01



//   VelyPID.KP=PID_data2[0];//可更改
//   VelyPID.KI=PID_data2[1];//可更改
//   VelyPID.KD=PID_data2[2];//可更改
}

unsigned char BT_Flag = 0, num = 0; //无线调参参数
unsigned char Move_Flag = 0; // 行进参数
void WirelessParaAdj(u8 WirelessParametric) // 无线调参接收数据
{
   
    if( WirelessParametric == 0x46 ) // 0x46 = 'F'
    {
      BT_Flag = 1;
    }
    
     if( BT_Flag == 1 )
    {
      PID_data1[num] = WirelessParametric;
      num++;
      if( num == 15 )
      {
        DataJudge(); // 无线 PID 调参
        BT_Flag = 0;
        num = 0;
      }
    }
    
   
}
