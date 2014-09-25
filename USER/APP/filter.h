#ifndef _FILTER_H_
#define _FILTER_H_
#include "IMU.h"

extern void Prepare_Data(S_INT16_XYZ *acc_in, S_INT16_XYZ *acc_out);
extern void Prepare_Data2(T_float_angle *angle_in);
extern void Prepare_Data3(S_INT16_XYZ *acc_in, S_INT16_XYZ *gyro_in, S_INT16_XYZ *acc_out);


#endif

