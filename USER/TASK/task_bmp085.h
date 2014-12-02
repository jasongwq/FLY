#ifdef __cplusplus
       extern "C" {
#endif
#include "../../GLIB/PERIPHERAL/BMP085/bmp085.h"
u32 Alt_bmp=0;

int task_bmp085(void)
{
    extern tg_BMP085_TYPE bmp085;
    _SS
    Soft_I2c1_Init();
    BMP085_Init();
		BMP085_Calibrate();
		static int16_t temp_up=0;
		static int16_t temp_ut=0;

    while (1)
    {
//        WaitX(500);
//        BMP085_Read(&bmp085);
//				BMP085_Printf(&bmp085);

        BMP085_temperature_start();
        WaitX(5);
        temp_ut = BMP085_temperature_get();
        static int i;
        for (i = 0; i < 40; i++)
        {
            BMP085_pressure_start();
            WaitX(26);
//						temp_up=BMP085_pressure_get();
//						Calculate(temp_ut,temp_up,&bmp085);       // 计算温度气压和高度
            BMP085_Calculate(temp_up, &bmp085);
//						Alt_bmp=(u32)(bmp085.altitude*100);
            BMP085_Printf(&bmp085);
        }
    }
    _EE
}
#ifdef __cplusplus
        }
#endif
