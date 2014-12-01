#ifdef __cplusplus
       extern "C" {
#endif
#include "../../GLIB/PERIPHERAL/BMP085/bmp085.h"
float Alt_bmp=0;

int task_bmp085(void)
{
    extern tg_BMP085_TYPE bmp085;
    _SS
    Soft_I2c1_Init();
    BMP085_Init();
    while (1)
    {
        //WaitX(100);
        //BMP085_Read(&bmp085);
        static int16_t temp;
        BMP085_temperature_start();
        WaitX(5);
        temp = BMP085_temperature_get();
        static int i;
        for (i = 0; i < 40; i++)
        {
            BMP085_pressure_start();
            WaitX(26);
            BMP085_Calculate(temp, &bmp085);
						Alt_bmp=bmp085.altitude;
            //BMP085_Printf(&bmp085);
        }
    }
    _EE
}
#ifdef __cplusplus
        }
#endif
