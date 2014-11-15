#include "Control.h"
#include "Balance.h"
#include "pwm.h"
#include "usr_usart.h"
#include "math.h"
#include "motor.h"
#include "filter.h"
#include "delay.h"


#define RC_Yaw_P 20
#define RC_ROL_P 20
#define RC_PIT_P 20
//#define RC_Yaw_P
//#define RC_Yaw_P


#define RC_FUN_MIN  1200
#define RC_FUN_MAX  1800
#define RC_FUN_CNT  20
#define RC_FUN_ZERO 1000

static float rol_i = 0, pit_i = 0, yaw_i = 0, alt_i = 0;
PID PID_ROL, PID_PIT, PID_YAW, PID_ALT, PID_POS,
    PID_PID_1, PID_PID_2, PID_PID_3, PID_PID_4,
    PID_PID_5, PID_PID_6, PID_PID_7, PID_PID_8,
    PID_PID_9, PID_PID_10, PID_PID_11, PID_PID_12;


static T_float_angle *att_in;
static S_INT16_XYZ   *gyr_in;
//static S_INT16_XYZ   *acc_in;
static T_RC_Data     *rc_in ;
static T_Control     *ctl;
void Balance_Data(T_float_angle *Att_in, S_INT16_XYZ *Gyr_in, S_INT16_XYZ *Acc_in, T_RC_Data *Rc_in, T_Control *Ctl)
{
    att_in = Att_in;
    gyr_in = Gyr_in;
    //    acc_in = Acc_in;
    rc_in  = Rc_in ;
    ctl    = Ctl   ;
}
/*****************************************************
/CONTROL
/armed 1 解锁
       0 上锁
*****************************************************/
#define DEAD_BAND 15
#define INTEGRAL_WINDUP_R 4000
#define INTEGRAL_WINDUP_P 4000
#define INTEGRAL_WINDUP_Y 3000
#define INTEGRAL_WINDUP_A 400
//float Throttle2;// HROTTLE;
static u32 Throttle_OUT;
static u32 Throttle_IN;

int ALT_Control_Out;
extern u8 flag_ALT;
vs16 Alt_Error, Alt_Error_Last;
void ALT_Control(u16 ALT_Set)
{
    if (1 == flag_ALT)
    {
        extern u16 Alt_ultrasonic;

        flag_ALT = 0;
        //time = 0;
        Alt_Error = ALT_Set - Alt_ultrasonic;
        PID_ALT.pout = PID_ALT.P * Alt_Error;
        alt_i += Alt_Error;
        PID_ALT.iout = (PID_ALT.I / 100) * alt_i;
        if (PID_ALT.iout > INTEGRAL_WINDUP_A)
            PID_ALT.iout = INTEGRAL_WINDUP_A;
        else if (alt_i < -INTEGRAL_WINDUP_A)
            PID_ALT.iout = -INTEGRAL_WINDUP_A;
        //        static u32 currenttime = 0;
        //        u32 lasttime = 0;
        //        lasttime = currenttime;
        //        currenttime = SysTick_Clock();
        PID_ALT.dout = -PID_ALT.D * (Alt_Error_Last - Alt_Error) * (1000000 / 10000); // (currenttime - lasttime));

        Alt_Error_Last = Alt_Error;
        //PID_ALT.dout = PID_ALT.D * (acc_in->z-8192);
        if (ctl->ALT_ON_OFF && Alt_ultrasonic != 0)
        {
            PID_ALT.OUT = PID_ALT.pout + PID_ALT.iout + PID_ALT.dout;
        }
        else
        {
            PID_ALT.OUT = 0;
        }
    }
}
static T_float_angle angle;
void Yaw_Control(void)
{
    angle.yaw = att_in->yaw + (rc_in->YAW   - 1500) / RC_Yaw_P + (-(*yaw_just - 10));
    if (angle.yaw < -180)
        angle.yaw += 360;
    else if (angle.yaw > 180)
        angle.yaw -= 360;
    /*****************************************************
    /P
    *****************************************************/
    PID_YAW.pout = PID_YAW.P * angle.yaw;
    /*****************************************************
    /I
    *****************************************************/
    yaw_i += angle.yaw; //积分
    if (yaw_i > INTEGRAL_WINDUP_Y)
        yaw_i = INTEGRAL_WINDUP_Y;
    else if (yaw_i < -INTEGRAL_WINDUP_Y)
        yaw_i = -INTEGRAL_WINDUP_Y;
    PID_YAW.iout = PID_YAW.I * yaw_i;
    /*****************************************************
    /D
    *****************************************************/
    PID_YAW.dout = PID_YAW.D * gyr_in->z;
    /*****************************************************
    /PID
    *****************************************************/
    PID_YAW.OUT = PID_YAW.pout + PID_YAW.iout + PID_YAW.dout;
}
void Rol_Control(void)
{
    angle.rol = att_in->rol - (rc_in->ROLL  - 1500) / RC_ROL_P + (-(*rol_just - 10));
    /*****************************************************
    /P
    *****************************************************/
    PID_ROL.pout = PID_ROL.P * angle.rol;
    /*****************************************************
    /I
    *****************************************************/
    rol_i += angle.rol; //积分
    if (rol_i > INTEGRAL_WINDUP_R)
        rol_i = INTEGRAL_WINDUP_R;
    else if (rol_i < -INTEGRAL_WINDUP_R)
        rol_i = -INTEGRAL_WINDUP_R;
    PID_ROL.iout = PID_ROL.I * rol_i;
    /*****************************************************
    /D
    *****************************************************/
    PID_ROL.dout = PID_ROL.D * gyr_in->x;
    /*****************************************************
    /PID
    *****************************************************/
    PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout - PID_ROL.dout;
}
void Pit_Control(void)
{
    angle.pit = att_in->pit + (rc_in->PITCH - 1500) / RC_PIT_P + (+(*pit_just - 10));
    /*****************************************************
    /P
    *****************************************************/
    PID_PIT.pout = PID_PIT.P * angle.pit;
    /*****************************************************
    /I
    *****************************************************/
    pit_i += angle.pit; //积分
    if (pit_i > INTEGRAL_WINDUP_P)
        pit_i = INTEGRAL_WINDUP_P;
    else if (pit_i < -INTEGRAL_WINDUP_P)
        pit_i = -INTEGRAL_WINDUP_P;
    PID_PIT.iout = PID_PIT.I * pit_i;
    /*****************************************************
    /D
    *****************************************************/
    PID_PIT.dout = PID_PIT.D * gyr_in->y;
    /*****************************************************
    /PID
    *****************************************************/
    PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
}
void Autoland(void)
{
    static u16 alt_tmp[13];
    static SLIDE_FILTERING16 alt_control = {alt_tmp, 0, sizeof(alt_tmp) / sizeof(alt_tmp[0]), 0, 0};
    alt_control.data = PID_ALT.OUT;
    slide_filtering16(alt_control);
}
void Balance(T_float_angle *att_in, S_INT16_XYZ *gyr_in, S_INT16_XYZ *acc_in, T_RC_Data *Rc_in, T_Control *Ctl)
{
    Balance_Data(att_in, gyr_in, acc_in, Rc_in, Ctl);
    Throttle_IN = rc_in->THROTTLE - RC_FUN_ZERO;
    Throttle_OUT = Throttle_IN;
    Yaw_Control();
    Rol_Control();
    Pit_Control();

    u16 ALT_Set = rc_in->AUX2 - 1000;

    if (0 == ctl->ALT_ON_OFF)
    {
        {
#define Balance_ALT 500
            static int Balance_Throttle = 0;
            static int i = 0;
            static u16 alt_tmp[80];
            static SLIDE_FILTERING16 alt_control = {alt_tmp, 0, sizeof(alt_tmp) / sizeof(alt_tmp[0]), 0, 0};
            alt_control.data = PID_ALT.OUT;//PID_ALT.OUT;
            slide_filtering16(alt_control);
            i++;
            if (100 == i)
            {
                Balance_Throttle = slide_filtering16(alt_control) - 10;
            }
            else if (i > 100)
            {
                extern u16 Alt_ultrasonic;
                if ((Alt_ultrasonic - (Balance_ALT - (i - 100) * 40)) > 0)
                {
                    Balance_Throttle -= 10;
                    Throttle_OUT += Balance_Throttle;
                }
                else if ((Alt_ultrasonic - (Balance_ALT - (i - 100) * 40)) < 0)
                {
                    Balance_Throttle += 10;
                    Throttle_OUT += Balance_Throttle;
                }
                else
                    Throttle_OUT += Balance_Throttle; //Throttle_OUT = Balance_Throttle;
            }
            else if (i <= 0)
            {
                i = -2;
            }
            else
            {
                ALT_Set = Balance_ALT;
                ALT_Control(ALT_Set);
                Throttle_OUT += PID_ALT.OUT;
            }

        }
    }
    else
    {
        ALT_Control(ALT_Set);
        Throttle_OUT += PID_ALT.OUT;
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
    //      s16 rolsinjust=sin(att_in->rol)*5;
    //      s16 pitsinjust=sin(att_in->pit)*5;
    //      if(rolsinjust<0)rolsinjust=-rolsinjust;
    //      if(pitsinjust<0)pitsinjust=-rolsinjust;

    //Throttle_OUT = Throttle_OUT * (rolsinjust+pitsinjust + 1);
    if (rc_in->THROTTLE > RC_FUN_MIN && ctl->ARMED)
    {
        MOTO1_PWM = (int32_t)((int)Throttle_OUT - PID_ROL.OUT + PID_PIT.OUT - PID_YAW.OUT);
        MOTO2_PWM = (int32_t)((int)Throttle_OUT + PID_ROL.OUT + PID_PIT.OUT + PID_YAW.OUT);
        MOTO3_PWM = (int32_t)((int)Throttle_OUT + PID_ROL.OUT - PID_PIT.OUT - PID_YAW.OUT);
        MOTO4_PWM = (int32_t)((int)Throttle_OUT - PID_ROL.OUT - PID_PIT.OUT + PID_YAW.OUT);
    }
    else
    {
        att_in->yaw = 0;
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














