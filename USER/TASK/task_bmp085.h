#ifdef __cplusplus
       extern "C" {
#endif
#include "../../GLIB/PERIPHERAL/BMP085/bmp085.h"
s32 Alt_bmp=0;
//#define I2c_Write      Soft_I2c1_Write
//#define I2c_ReadBuffer Soft_I2c1_ReadBuffer
//#define I2c_Init       Soft_I2c1_Init
int task_bmp085(void)
{
    extern tg_BMP085_TYPE bmp085;
    _SS
    Soft_I2c1_Init();
    BMP085_Init();
		BMP085_Calibrate();
		static int32_t temp_up=0;
		static int32_t temp_ut=0;

    while (1)
    {
//        WaitX(100);
//        BMP085_Read(&bmp085);
//		    BMP085_Printf(&bmp085);
        BMP085_temperature_start();
        WaitX(5);
        temp_ut = BMP085_temperature_get();
        static int i;
        for (i = 0; i < 1; i++)
        {
            BMP085_pressure_start();
            WaitX(5);
						temp_up=BMP085_pressure_get();
						Calculate(temp_ut,temp_up,&bmp085);       // 计算温度气压和高度
						
						Alt_bmp=(s32)(bmp085.altitude*1000);
//            BMP085_Printf(&bmp085);
        }
    }
    _EE
}
#ifdef __cplusplus
        }
#endif
