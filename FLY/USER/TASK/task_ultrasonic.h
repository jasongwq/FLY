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
u16 Alt_temp_l;


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
        Alt_ultrasonic1t = Alt_ultrasonic1;
        Alt_ultrasonic2t = Alt_ultrasonic2;
        Alt_temp = Alt_ultrasonic1 >= Alt_ultrasonic2 ? Alt_ultrasonic1 : Alt_ultrasonic2;
				if(Alt_temp==0||Alt_temp>2000)Alt_temp=Alt_temp_l;
				else Alt_temp_l=Alt_temp;
        {
            static s16 alt_tmp[8] = {0};
            static SLIDE_FILTERING16 alt_ultrasonic_fil = {alt_tmp, 0, sizeof(alt_tmp) / sizeof(alt_tmp[0]), 0, 0};
            alt_ultrasonic_fil.data = (s16)Alt_temp;
            Alt_ultrasonic = slide_filtering16(&alt_ultrasonic_fil);
        }
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
                if (0 == flag_ALT1)
                {
                    Alt_ultrasonic1 = 0;
                }
                if (1 == flag_ALT2)
                {
                    Alt_ultrasonic2 = 0;
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
