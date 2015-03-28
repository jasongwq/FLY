#ifdef __cplusplus
       extern "C" {
#endif
#include "data_transfer.h"
#include "control.h"
#include "pwm.h"
#define EX_BaudRate 8
int task_pwm_ex(void)
{
    _SS
    Ex_Init();
    TIM2_PWM_Init(5000, 36);
    TIM2_PWM_Init(5000, 36);
    TIM8_PWM_Init(5000, 36);
    TIM8_PWM_Init(5000, 36);
    TIM_SetCompare1(TIM2, 2300);
    TIM_SetCompare2(TIM2, 2300);
    TIM_SetCompare1(TIM8, 2300);
    TIM_SetCompare2(TIM8, 2300);

    //    exfuns_init();          //为fatfs相关变量申请内存
    //    f_mount(0, fxs[0]);     //挂载SD卡
    //    while (SD_Init() != SD_OK)    //检测SD卡
    //    {
    //        ;//WaitX(200);
    //    }
    //    static char srtc_path[25];
    //    while (1)
    //    {
    //        WaitX(2000);

    //        int fatbuf_Offset = 0;
    //        int sum = 0;
    //        //time
    //        sum = sprintf((char *)fatbuf + (fatbuf_Offset += 0), "\r\n");
    //        sum = sprintf((char *)fatbuf + (fatbuf_Offset += sum), "%8d", ((pcalendar.w_date * 100 + pcalendar.hour) * 100 + pcalendar.min) * 100 + pcalendar.sec);
    //        //DHT11
    //        sum = sprintf((char *)fatbuf + (fatbuf_Offset += sum), " %2d", Temperature);
    //        sum = sprintf((char *)fatbuf + (fatbuf_Offset += sum), " %2d", humidity);
    //        //Angle
    //        sum = sprintf((char *)fatbuf + (fatbuf_Offset += sum), " %4d %+4d", Steering_Angle.x, Angle_Offset.x);
    //        sum = sprintf((char *)fatbuf + (fatbuf_Offset += sum), " %4d %+4d", Steering_Angle.y, Angle_Offset.y);
    //        //AD数据
    //        sum = sprintf((char *)fatbuf + (fatbuf_Offset += sum), " %4d", Radiometer_Data->s);
    //        sum = sprintf((char *)fatbuf + (fatbuf_Offset += sum), " %4d", Radiometer_Data->x);
    //        sum = sprintf((char *)fatbuf + (fatbuf_Offset += sum), " %4d", Radiometer_Data->z);
    //        sum = sprintf((char *)fatbuf + (fatbuf_Offset += sum), " %4d", Radiometer_Data->y);
    //        sum = sprintf((char *)fatbuf + (fatbuf_Offset += sum), " %4d", Radiometer_Data->c);
    //        f_write(file, (char *)fatbuf, (fatbuf_Offset += sum), &bxw);
    //        f_sync(file);
    //    }
    //    //f_close(file);
    //    _EE
    //}

    while (1)
    {
        if (Ex_ON_OFF.Status)
        {
            WaitX(EX_BaudRate);
            Send.Status = 1;
            Data_Exchange();
        }
        if (Ex_ON_OFF.Senser)
        {
            WaitX(EX_BaudRate);
            Send.Senser = 1;
            Data_Exchange();
        }
        if (Ex_ON_OFF.RCData)
        {
            WaitX(EX_BaudRate);
            Data_Exchange();
        }
        if (Ex_ON_OFF.MotoPwm)
        {
            WaitX(EX_BaudRate);
            Send.MotoPwm = 1;
            Data_Exchange();
        }
        if (Ex_ON_OFF.DataF1)
        {
            WaitX(EX_BaudRate);
            Send.DataF1 = 1;
            Data_Exchange();
        }
        if (Ex_ON_OFF.DataF2)
        {
            WaitX(EX_BaudRate);
            Send.DataF2 = 1;
            Data_Exchange();
        }
        if (Ex_ON_OFF.DataF3)
        {
            WaitX(EX_BaudRate);
            Send.DataF3 = 1;
            Data_Exchange();
        }
        if (Ex_ON_OFF.DataF4)
        {
            WaitX(EX_BaudRate);
            Send.DataF4 = 1;
            Data_Exchange();
        }
        WaitX(EX_BaudRate);
        Data_Exchange();
    }
    _EE
}
//#undef EX_BaudRate
#ifdef __cplusplus
        }
#endif
