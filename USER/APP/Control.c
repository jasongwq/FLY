#include "Control.h"
//#include "pwm.h"
//#include "usr_usart.h"
//#include "math.h"


//void Control(T_float_angle *att_in, S_INT16_XYZ *gyr_in, T_RC_Data *rc_in, T_Control *ctl)
//{
//    /*****************************************************
//    /定高
//    *****************************************************/
//    ctl->ALT_ON_OFF = 0;
//    /*****************************************************
//    /ALT
//    *****************************************************/
//    if (ctl->ALT_ON_OFF)
//    {
//        static int time = 0;
//        time++;
//        Alt_Set = (s16)PID_PID_2.I;//(s16)(PID_PID_10.I * 100);
//        if (time > 15)
//        {
//            time = 0;
//            Alt_Error = Alt_Set - Alt_ultrasonic;
//            PID_ALT.pout = PID_ALT.P * Alt_Error;
//            alt_i += Alt_Error; 
//            PID_ALT.iout = (PID_ALT.I / 100) * alt_i;
//            if (PID_ALT.iout > INTEGRAL_WINDUP_A)
//                PID_ALT.iout = INTEGRAL_WINDUP_A;
//            else if (alt_i < -INTEGRAL_WINDUP_A)
//                PID_ALT.iout = -INTEGRAL_WINDUP_A;
//            PID_ALT.dout = -PID_ALT.D * (Alt_Error_Last - Alt_Error);
//            Alt_Error_Last = Alt_Error;
//        }
//        //     //PID_ALT.dout =  ((Average_Acc.z - 7410) * PID_ALT.D);//
//        PID_ALT.out  = PID_ALT.pout + PID_ALT.iout + PID_ALT.dout;
//        Throttle2 = Throttle + PID_ALT.out;
//        if (Throttle2 < 0)
//            Throttle2 = 0;
//    }
//    else
//    {
//        Throttle2 = Throttle;//
//    }
//}
