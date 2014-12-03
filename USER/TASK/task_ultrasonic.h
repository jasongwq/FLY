#ifdef __cplusplus
extern "C" {
#endif

#include "ultrasonic.h"
#include "pfilter.h"

u16 Alt_ultrasonic;
u16 Alt_ultrasonic1;
u16 Alt_ultrasonic2;
u16 Alt_ultrasonic1t;
u16 Alt_ultrasonic2t;

u16 Alt_temp;

u8 flag_ALT;
u8 flag_ALT1;
u8 flag_ALT2;
int task_ultrasonic(void)
{
    u32 temp = 0;
    _SS
    Ultrasonic_Init();
    while (1)
    {
        static u8 utime;
				Alt_ultrasonic1t=Alt_ultrasonic1;
				Alt_ultrasonic2t=Alt_ultrasonic2;
        Alt_ultrasonic = Alt_ultrasonic1 >= Alt_ultrasonic2 ? Alt_ultrasonic1 : Alt_ultrasonic2;
        flag_ALT = 1; flag_ALT2 = flag_ALT1 = 0;
        Alt_ultrasonic1 = Alt_ultrasonic2 = 0;
        WaitX(30 - utime);
        TRIG1_H;
        TRIG2_H;
        TIM1_CAPTURE_STA_CH1 = 0;
        TIM1_CAPTURE_STA_CH2 = 0;
        TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0
        TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0
        WaitX(1);
        TRIG1_L;
        TRIG2_L;
        utime = 0;
        while (1)
        {
            WaitX(1);
            if ((TIM1_CAPTURE_STA_CH1 & 0X80) && (0 == flag_ALT1))
            {
                temp  = TIM1_CAPTURE_STA_CH1 & 0X3F;
                temp *= 65536;
                temp = TIM1_CAPTURE_VAL_CH1;
                Alt_ultrasonic1 = (temp * 340 / 1000 / 2);//mm
                flag_ALT1 = 1;
            }
            if ((TIM1_CAPTURE_STA_CH2 & 0X80) && (0 == flag_ALT2))
            {
                temp  = TIM1_CAPTURE_STA_CH2 & 0X3F;
                temp *= 65536;
                temp = TIM1_CAPTURE_VAL_CH2;
                Alt_ultrasonic2 = (temp * 340 / 1000 / 2);//mm
                flag_ALT2 = 1;
            }
            if (flag_ALT2 && flag_ALT1)
            {
                flag_ALT2 = flag_ALT1 = 0;
                break;
            }
            utime++;
            if (utime > 20)
            {
                if (0==flag_ALT1)
                {
                    Alt_ultrasonic1=0;
                }
                if (1==flag_ALT2)
                {
                    Alt_ultrasonic2=0;
                }
                //Alt_ultrasonic1 = Alt_ultrasonic2 = Alt_ultrasonic = 0;
                break;
            }
        }
    }
    _EE
}
#ifdef __cplusplus
}
#endif
