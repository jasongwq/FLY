#ifndef __BALANCE_H_
#define __BALANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sys.h"
#include "data_transfer.h"
#include "IMU.h"
#include "rc.h"

extern int32_t MOTO1_PWM, MOTO2_PWM, MOTO3_PWM, MOTO4_PWM;


//extern void Balance_Data(T_float_angle *att_in,S_INT16_XYZ *gyr_in, T_RC_Data *rc_in,T_Control *ctl_data);
extern void Balance(T_float_angle *att_in, S_INT16_XYZ *gyr_in, S_INT16_XYZ *acc_in, T_RC_Data *rc_in, T_Control *ctl);
extern int Balance_Throttle;
//extern void Balance(void);

#ifdef __cplusplus
        }
#endif

#endif


