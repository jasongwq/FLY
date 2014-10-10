#ifndef __BALANCE_H_
#define __BALANCE_H_
#include "sys.h"
#include "data_transfer.h"
#include "IMU.h"
#include "rc.h"

extern int32_t MOTO1_PWM, MOTO2_PWM, MOTO3_PWM, MOTO4_PWM;

typedef struct {float P,pout,I,iout,D,dout,IMAX,OUT;}PID;

extern PID PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_POS,PID_PID_1,PID_PID_2;

//extern void Balance_Data(T_float_angle *att_in,S_INT16_XYZ *gyr_in, T_RC_Data *rc_in,T_Control *ctl_data);
extern void Balance_Data(T_float_angle *Att_in,S_INT16_XYZ *Gyr_in, S_INT16_XYZ *Acc_in, T_RC_Data *Rc_in, T_Control *Ctl);

extern void Balance(void);


#endif


