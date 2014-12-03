#ifdef __cplusplus
       extern "C" {
#endif

#include "ultrasonic.h"
#include "pfilter.h"

u16 Alt_ultrasonic;
u16 Alt_temp;

u8 flag_ALT;
int task_ultrasonic(void)
{
    u32 temp = 0;
    _SS
    Ultrasonic_Init();
    while (1)
    {
        static u8 utime;
        WaitX(30 - utime);
        TRIG_H;
        TIM1_CAPTURE_STA_CH1 = 0;
        TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0
        WaitX(1);
        TRIG_L;
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
//								static s16 alt_tmp[2]={0};
//                static SLIDE_FILTERING16 alt_ultrasonic_fil = {alt_tmp, 0, sizeof(alt_tmp) / sizeof(alt_tmp[0]), 0, 0};
//                alt_ultrasonic_fil.data = (s16)Alt_temp;
//                Alt_ultrasonic = slide_filtering16(&alt_ultrasonic_fil);
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
