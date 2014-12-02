#ifdef __cplusplus
       extern "C" {
#endif
#include "../../GLIB/PERIPHERAL/BMP085/bmp085.h"
u32 Alt_bmp=0;
#define I2c_Write      Soft_I2c1_Write
#define I2c_ReadBuffer Soft_I2c1_ReadBuffer
#define I2c_Init       Soft_I2c1_Init
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
//        //WaitX(100);
////        BMP085_Read(&bmp085);
//		BMP085_Printf(&bmp085);
//    //temp_ut = bmp085ReadTemp();      // 读取温度
//		BMP085_temperature_start();
//        WaitX(5);
//        temp_ut = BMP085_temperature_get();
//				           BMP085_pressure_start();
//            WaitX(5);
//						temp_up=BMP085_pressure_get();
////   //temp_up = bmp085ReadPressure();  // 读取压强
////	 {
////    //int32_t pressure = 0;
//////    I2c_Write(BMP085_Addr, 0xF4, 0x34);
//////    delay_ms(5); // max time is 4.5ms
////    uint8_t tmp[2];
////    I2c_ReadBuffer(BMP085_Addr, 0xF6, 2, tmp); //读出2个数据
////    temp_up = (int16_t)( (tmp[0] << 8) + tmp[1]  );
////    temp_up &= 0x0000FFFF; //不能去掉 原因未知
////    //return pressure;
////}
//    Calculate(temp_ut, temp_up,  &bmp085); //计算结果放入结构体
        BMP085_temperature_start();
        WaitX(5);
        temp_ut = BMP085_temperature_get();
        static int i;
        for (i = 0; i < 1; i++)
        {
            BMP085_pressure_start();
            WaitX(5);
						//temp_up=BMP085_pressure_get();
						Calculate(temp_ut,temp_up,&bmp085);       // 计算温度气压和高度
            //BMP085_Calculate(temp_ut, &bmp085);
//						Alt_bmp=(u32)(bmp085.altitude*100);
						//WaitX(100);
            BMP085_Printf(&bmp085);
        }
    }
    _EE
}
#ifdef __cplusplus
        }
#endif
