#ifndef __IMU_H_
#define __IMU_H_
#ifdef __cplusplus
       extern "C" {
#endif
#include "MPU6050.h"
typedef struct
{
    float rol;
    float pit;
    float yaw;
} T_float_angle;


extern void IMUupdate(S_INT16_XYZ *acc, S_INT16_XYZ *gyr, T_float_angle *angle);
extern T_float_angle    Att_Angle;  //ATT函数计算出的姿态角
//extern T_float_angle    Att_Angle_Avg;  //ATT函数计算出的姿态角
extern float   AngleOffset_Rol, AngleOffset_Pit;
extern float position_x,position_y,position_z;
#ifdef __cplusplus
        }
#endif
#endif


