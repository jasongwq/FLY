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
        WaitX(30 - utime);
        TRIG1_H;
        TIM1_CAPTURE_STA_CH1 = 0;
        TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0
        WaitX(1);
        TRIG1_L;
        utime = 0;
        while (1)
        {
            WaitX(1);
            if (TIM1_CAPTURE_STA_CH1 & 0X80)
            {
                temp  = TIM1_CAPTURE_STA_CH1 & 0X3F;
                temp *= 65536;
                temp = TIM1_CAPTURE_VAL_CH1;
                Alt_ultrasonic = (temp * 340 / 1000 / 2);//mm
                flag_ALT=1;
                break;
            }
            utime++;
            if (utime > 10)
            {
                Alt_ultrasonic = 0;
                break;
            }
        }
    }
    _EE
}
#ifdef __cplusplus
}
#endif
