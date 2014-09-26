#include "Control.h"
#include "pwm.h"
#include "usr_usart.h"


#define Moto_PwmMax 1000
static void Moto_PwmRflash(int32_t MOTO1_PWM, int32_t MOTO2_PWM, int32_t MOTO3_PWM, int32_t MOTO4_PWM)
{
    if (MOTO1_PWM > Moto_PwmMax)   MOTO1_PWM = Moto_PwmMax;
    if (MOTO2_PWM > Moto_PwmMax)   MOTO2_PWM = Moto_PwmMax;
    if (MOTO3_PWM > Moto_PwmMax)   MOTO3_PWM = Moto_PwmMax;
    if (MOTO4_PWM > Moto_PwmMax)   MOTO4_PWM = Moto_PwmMax;
    if (MOTO1_PWM < 0) MOTO1_PWM = 0;
    if (MOTO2_PWM < 0) MOTO2_PWM = 0;
    if (MOTO3_PWM < 0) MOTO3_PWM = 0;
    if (MOTO4_PWM < 0) MOTO4_PWM = 0;
    TIM_SetCompare1(TIM2, MOTO1_PWM + 2300);
    TIM_SetCompare2(TIM2, MOTO2_PWM + 2300);
    TIM_SetCompare1(TIM8, MOTO3_PWM + 2300);
    TIM_SetCompare2(TIM8, MOTO4_PWM + 2300);
}
extern u16 Alt_ultrasonic;
int32_t MOTO1_PWM, MOTO2_PWM, MOTO3_PWM, MOTO4_PWM;
PID PID_ROL, PID_PIT, PID_YAW, PID_ALT, PID_POS, PID_PID_1, PID_PID_2;
float rol_i = 0, pit_i = 0, yaw_i = 0, alt_i = 0;
s32 Alt_Error;
s32 Alt_LastError;

/*****************************************************
/CONTROL
/armed 1 解锁
       0 上锁
*****************************************************/
#define DEAD_BAND 15
#define INTEGRAL_WINDUP_R 4000
//((1000-(rc_in->THROTTLE - RC_FUN_ZERO))/2/PID_ROL.I)
#define INTEGRAL_WINDUP_P 4000
//((1000-(rc_in->THROTTLE - RC_FUN_ZERO))/2/PID_PIT.I)


void Control(T_float_angle *att_in, S_INT16_XYZ *gyr_in, T_RC_Data *rc_in, T_Control *ctl)
{
    T_float_angle angle;
    static u8 Last_Constant_Level = 0;
    static s32 Alt_Control;
    static char alt_time = 0;

    angle.yaw = att_in->yaw + (rc_in->YAW   - 1500) / 60 + (+(PID_PID_1.P - 10));
    if (angle.yaw < -180)
        angle.yaw += 360;
    else if (angle.yaw > 180)
        angle.yaw -= 360;

    angle.rol = att_in->rol - (rc_in->ROLL  - 1500) / 60 + (-(PID_PID_1.I - 10));
    angle.pit = att_in->pit + (rc_in->PITCH - 1500) / 60 + (+(PID_PID_1.D - 10));
    /*****************************************************
    /定高
    *****************************************************/
    if (alt_time++ > 15)
    {
        alt_time = 0;
        if (ctl->Constant_Level || Alt_ultrasonic)
        {
            if (0 == Last_Constant_Level)
            {
                Alt_Control = Alt_ultrasonic;
            }
            else
            {
                Alt_LastError = Alt_Error;
                Alt_Error = (Alt_Control - Alt_ultrasonic);

            }
        }
        else
        {
            Alt_Control = 0;
            Alt_Error = 0;
        }//Sys_Printf(USART1, " %d -%d = %d \r\n",Alt_Control,Alt_ultrasonic, Alt_Error);
        Last_Constant_Level = ctl->Constant_Level;
    }

    /*****************************************************
    /P
    *****************************************************/
    PID_ROL.pout = PID_ROL.P * angle.rol;
    PID_PIT.pout = PID_PIT.P * angle.pit;
    PID_YAW.pout = PID_YAW.P * angle.yaw;
    PID_ALT.pout = PID_ALT.P * Alt_Error;

    /*****************************************************
    /I
    *****************************************************/
    rol_i += angle.rol; //积分
    if (rol_i > INTEGRAL_WINDUP_R)
        rol_i = INTEGRAL_WINDUP_R;
    else if (rol_i < -INTEGRAL_WINDUP_R)
        rol_i = -INTEGRAL_WINDUP_R;

    pit_i += angle.pit; //积分
    if (pit_i > INTEGRAL_WINDUP_P)
        pit_i = INTEGRAL_WINDUP_P;
    else if (pit_i < -INTEGRAL_WINDUP_P)
        pit_i = -INTEGRAL_WINDUP_P;

    //if (angle.rol < DEAD_BAND && angle.rol > -DEAD_BAND && angle.pit > -DEAD_BAND && angle.pit < DEAD_BAND)
    yaw_i += angle.yaw; //积分
    if (yaw_i > 3000)
        yaw_i = 3000;
    else if (yaw_i < -3000)
        yaw_i = -3000;

    alt_i += Alt_Error; //积分
    if (alt_i > 3000)
        alt_i = 3000;
    else if (alt_i < -3000)
        alt_i = -3000;

    PID_ROL.iout = PID_ROL.I * rol_i;
    PID_PIT.iout = PID_PIT.I * pit_i;
    PID_YAW.iout = PID_YAW.I * yaw_i;
    PID_ALT.iout = PID_ALT.I * alt_i;

    /*****************************************************
    /D
    *****************************************************/
    PID_ROL.dout = PID_ROL.D * gyr_in->x;
    PID_PIT.dout = PID_PIT.D * gyr_in->y;
    PID_YAW.dout = PID_YAW.D * gyr_in->z;

    if (alt_time == 0)
        PID_ALT.dout = PID_ALT.D * (Alt_LastError - Alt_Error) / 0.030;
    /*****************************************************
    /PID
    *****************************************************/
    PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout - PID_ROL.dout;
    PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
    PID_YAW.OUT = PID_YAW.pout + PID_YAW.iout + PID_YAW.dout;

    if (ctl->Constant_Level)
    {
        PID_ALT.OUT = PID_ALT.pout + PID_ALT.iout - PID_ALT.dout;
    }
    else
    {
        PID_ALT.OUT = 0;
    }
    /*****************************************************
    /CONTROL
    *****************************************************/
    //              ROL
    //               +
    // MOTO1_PWM    /|\    MOTO1_PWM
    //               |
    //        + ----------- - PITCH
    //               |
    // MOTO1_PWM     |     MOTO1_PWM
    //               -
    //
    //if (angle.rol > DEAD_BAND || angle.rol < -DEAD_BAND || angle.pit > DEAD_BAND || angle.pit < -DEAD_BAND)
    //    PID_YAW.OUT = 0;

    if (rc_in->THROTTLE > RC_FUN_MIN && ctl->ARMED)
    {
        MOTO1_PWM = rc_in->THROTTLE - RC_FUN_ZERO - PID_ROL.OUT + PID_PIT.OUT + PID_ALT.OUT - PID_YAW.OUT;
        MOTO2_PWM = rc_in->THROTTLE - RC_FUN_ZERO + PID_ROL.OUT + PID_PIT.OUT + PID_ALT.OUT + PID_YAW.OUT;
        MOTO3_PWM = rc_in->THROTTLE - RC_FUN_ZERO + PID_ROL.OUT - PID_PIT.OUT + PID_ALT.OUT - PID_YAW.OUT;
        MOTO4_PWM = rc_in->THROTTLE - RC_FUN_ZERO - PID_ROL.OUT - PID_PIT.OUT + PID_ALT.OUT + PID_YAW.OUT;
    }
    else
    {
        pit_i = 0;
        rol_i = 0;
        yaw_i = 0;
        alt_i = 0;
        MOTO1_PWM = 0;
        MOTO2_PWM = 0;
        MOTO3_PWM = 0;
        MOTO4_PWM = 0;
    }
    Moto_PwmRflash(MOTO1_PWM, MOTO2_PWM, MOTO3_PWM, MOTO4_PWM);
}
