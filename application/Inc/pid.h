#ifndef _PID_H
#define _PID_H

#include "main.h"

/* PID结构体 */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float IntegralLimit;
    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    float dt;

} PIDInstance;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float IntegralLimit; // 积分限幅

} PID_Init_Config_s;

void PIDInit(PIDInstance *pid, PID_Init_Config_s *config, float dt);
float PIDCalculate(PIDInstance *pid, float measure, float ref);
void PIDClear(PIDInstance *pid);

#endif // !_PID_H