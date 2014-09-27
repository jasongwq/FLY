#ifndef __BALANCE_H_
#define __BALANCE_H_
#include "sys.h"
#include "data_transfer.h"
#include "IMU.h"
#include "rc.h"


typedef struct {float P,pout,I,iout,D,dout,IMAX,OUT;}PID;

extern PID PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_POS,PID_PID_1,PID_PID_2;

extern void Control(T_float_angle *att_in,S_INT16_XYZ *gyr_in, T_RC_Data *rc_in,T_Control *ctl_data);


#endif


