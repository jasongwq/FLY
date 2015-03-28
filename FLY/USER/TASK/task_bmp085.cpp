/***SYS***/
#include "sys.h"
#include "gpio.h"
#include <string.h>
#include "usr_usart.h"
#include "minos_delay.h"
#include "sys_os.h"
#include "gpio.h"
#include "stm32_config.h"
#include "G32_timer.h"
/***C***/
#include "string.h"
/***Other***/

#include "pfilter.h"

#include "kalman.h"

#include "../../GLIB/PERIPHERAL/BMP085/bmp085.h"
s32 Alt_bmp = 0;
s32 Alt_bmp1 = 0;
s32 Alt_bmp2 = 0;

/*
    Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
    R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
*/
extern float *KALMAN_Q ;
extern float *KALMAN_R ;
extern float *KALMAN_P0;
int task_bmp085(void)
{
    extern tg_BMP085_TYPE bmp085;
    _SS
    Soft_I2c1_Init();
    BMP085_Init();
    BMP085_Calibrate();
    static int32_t temp_up = 0;
    static int32_t temp_ut = 0;

    while (1)
    {
        //        WaitX(100);
        //        BMP085_Read(&bmp085);
        //          BMP085_Printf(&bmp085);
        BMP085_temperature_start();
        WaitX(5);
        temp_ut = BMP085_temperature_get();
        static int i;
        for (i = 0; i < 1; i++)
        {
            BMP085_pressure_start();
            WaitX(5);
            temp_up = BMP085_pressure_get();
            Calculate(temp_ut, temp_up, &bmp085);     // 计算温度气压和高度
            Alt_bmp2 = (s32)(bmp085.altitude * 1000+1000);
//            {
//                static s32 Alt_bmp_tmp[16] = {0};
//                static SLIDE_FILTERING32 Alt_bmp_fil = {Alt_bmp_tmp, &Alt_bmp2, sizeof(Alt_bmp_tmp) / sizeof(Alt_bmp_tmp[0]), 0, 0};
//                Alt_bmp1 = slide_filtering32(&Alt_bmp_fil);
//            }
            {
            Alt_bmp=KalmanFilter((double)Alt_bmp2, *KALMAN_Q, *KALMAN_R, *KALMAN_P0,0);
            }
            //            BMP085_Printf(&bmp085);
        }
    }
    _EE
}
