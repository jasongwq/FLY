#ifndef __USER_H
#define __USER_H

#ifdef __cplusplus
       extern "C" {
#endif
#include "sys.h"
#include "gpio.h"
#include "pid.h"
#include "usr_usart.h"
struct Parameter
{
    u16 up;//设定上限
    u16 down;//设定下限
    u16 powerd;//实际功率
    u16 powerf;//实际功率
    u16 setpower;//设定功率
    u16 speed;//实际速度
    u16 setspeed;//实际速度
    u16 flag;//标志位
    u16 settemperature;//设定温度1
    u16 settemperature2;//设定温度2
    s16 fenshan;//风速PWM
    s16 dianzu;//电阻PWM
    u16 time_flag;
    u16 last_flag;//模式
};
extern struct Parameter parameter;
//struct PID
//{
//    s16 SetPoint; // 设定目标 Desired Value
//    s16 Proportion; // 比例常数 Proportional Const
//    s16 Integral; // 积分常数 Integral Const
//    s16 Derivative; // 微分常数 Derivative Const
//    s32 LastError; // Error[-1]
//    s32 PrevError; // Error[-2]
//    s32 SumError; // Sums of Errors
//};

extern struct PID spid; // PID Control Structure
extern s8 FixedCurve[][210];
extern int mode2unm;
extern int Curve;
#ifdef __cplusplus
        }
#endif
#endif
