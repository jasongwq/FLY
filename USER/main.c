/***SYS***/
#include "user.h"
#include "sys.h"
#include "minos_delay.h"
#include "sys_os.h"
#include "gpio.h"
#include "stm32_config.h"
#include "G32_timer.h"
/***C***/
#include "string.h"
/***Other***/
void SYS_INIT(void)
{
    extern void NVIC_Configuration(void);
    /***延时初始化***/
    delay_init();
    /***中断初始化***/
    NVIC_Configuration();
    uart_init (115200);
    Sys_Printf(USART1, "\r\nUSART1 ok");
    delay_ms(500);
}
#include "usr_usart.h"
#include "MPU6050.h"
#include "IMU.h"
#include "stdio.h"
#include "filter.h"
#include "IMU.h"
#include "Control.h"
#include "rc.h"
#include "bak.h"

#define LED1_Init     PA2_OUT
#define LED1_Toggle {static char i = 0;PAout(2) = i++&0X01;}
#define LED1_H PAout(2) = 1;
#define LED1_L PAout(2) = 0;
int task_led(void)
{
    _SS
    LED1_Init;
    LED1_H;
    while (1)
    {
        WaitX(100);
        if (RC_Control.ARMED)
            LED1_H
        else
            LED1_Toggle
    }
    _EE
}
#include "ultrasonic.h"
u16 Alt_ultrasonic;
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
        TIM1_CAPTURE_STA_CH1 = 0; //开启下一次捕获
        TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
        WaitX(1);
        TRIG_L;
        utime = 0;
        while (1)
        {
            WaitX(1);
            if (TIM1_CAPTURE_STA_CH1 & 0X80) //成功捕获到了一次上升沿
            {
                temp  = TIM1_CAPTURE_STA_CH1 & 0X3F;
                temp *= 65536;//溢出时间总和
                temp = TIM1_CAPTURE_VAL_CH1; //得到总的高电平时间
                //Sys_Printf(USART1, "%dMM\r\n", temp); //打印总的高点平时间
                //Sys_Printf(USART1, "%dMM\r\n", (temp * 340 / 1000 / 2)); //打印总的高点平时间
                Alt_ultrasonic = (temp * 340 / 1000 / 2);
                //Sys_Printf(USART1, "%dMM\r\n", Alt_ultrasonic);
                break;
            }
            utime++;
            if (utime > 10)
            {
                break;
            }
        }
    }
    _EE
}
#include "HMC5883L.h"
#define PI 3.14159265
extern float compass_yaw;
int task_hmc5883l(void)
{
    extern S_INT16_XYZ Mag;
    _SS
    HMC5883L_Init();
    HMC5883L_Calibrate();
    WaitX(20);
    while (1)
    {
        HMC5883L_Start();//7us
        WaitX(10);
        /**********430us*************/
        HMC5883L_MultRead(&hmc5883l);//8us
        //HMC5883L_Printf(&hmc5883l);//测试输出角度
        Mag.x = hmc5883l.hx;
        Mag.y = hmc5883l.hy;
        Mag.z = hmc5883l.hz;
        {//来自APM
            float angle1, angle2,
                  sin_angle1, sin_angle2,
                  cos_angle1, cos_angle2,
                  Xr, Yr;

            angle1 = (Att_Angle.rol + 180) / 180 * PI;
            angle2 = (-Att_Angle.pit + 180) / 180 * PI;
            sin_angle1 = sin(angle1);
            cos_angle1 = cos(angle1);
            sin_angle2 = sin(angle2);
            cos_angle2 = cos(angle2);
						
            Xr = Mag.x * cos_angle2 * cos_angle2 + Mag.y * sin_angle2 * sin_angle1 - Mag.z * cos_angle1 * sin_angle2;
            Yr = Mag.y * cos_angle1 + Mag.z * sin_angle1;
            compass_yaw = atan2(Yr, Xr) * (180 / PI); //160us计算时间
        }
        WaitX(10);
    }
    _EE
}

int flag_ACC = 0;

int task_6050(void)
{
    extern S_INT16_XYZ Acc, Gyr, Mag,Average_Acc;
    _SS
    MPU6050_Init();
    if (0 == Data_Read())
    {
        StmFlash_Read();
    }
    WaitX(200);
    while (1)
    {
        WaitX(2);
        MPU6050_Read();
        MPU6050_Dataanl();//5us
        Acc.x = MPU6050_ACC_LAST.x;
        Acc.y = MPU6050_ACC_LAST.y;
        Acc.z = MPU6050_ACC_LAST.z;
        Gyr.x = MPU6050_GYRO_LAST.x;
        Gyr.y = MPU6050_GYRO_LAST.y;
        Gyr.z = MPU6050_GYRO_LAST.z;
        Prepare_Data(&Acc, &Average_Acc);//4us
        Prepare_Data2(&Att_Angle);//24us
        flag_ACC = 1;
    }
    _EE
}
#include "data_transfer.h"
#include "control.h"
#include "pwm.h"
int task_pwm_ex(void)
{
    _SS
    //    TIM2_PWM_Init(5000, 36);
    //    TIM2_PWM_Init(5000, 36);
    //    TIM8_PWM_Init(5000, 36);
    //    TIM8_PWM_Init(5000, 36);
    //    TIM_SetCompare1(TIM2, 2300);
    //    TIM_SetCompare2(TIM2, 2300);
    //    TIM_SetCompare1(TIM8, 2300);
    //    TIM_SetCompare2(TIM8, 2300);
    while (1)
    {
        WaitX(25);
        Data_Exchange();
    }
    _EE
}
#include "G32_Timer_Cap.h"
extern int RC_flag_ok;
extern u8 RC_data_cnt;
extern u8 *p_RxBuffer;
extern void Data_Receive_Anl(u8 *data_buf, u8 num);

int task_cap_rc(void)
{
    static u16 Rc_Pwm_In[10];
    u32 temp = 0;
    static u16 capok;
    _SS
    TIM2_Cap_Init(0XFFFF, 72);
    TIM3_Cap_Init(0XFFFF, 72);
    TIM4_Cap_Init(0XFFFF, 72);
    TIM8_Cap_Init(0XFFFF, 72);
    while (1)
    {
        static u8 utime;
        WaitX(25 - utime * 2);
        TIM4_CAPTURE_STA_CH1 = 0; //开启下一次捕获
        TIM4_CAPTURE_STA_CH2 = 0; //开启下一次捕获
        TIM3_CAPTURE_STA_CH1 = 0; //开启下一次捕获
        TIM3_CAPTURE_STA_CH2 = 0; //开启下一次捕获
        TIM3_CAPTURE_STA_CH3 = 0; //开启下一次捕获
        TIM3_CAPTURE_STA_CH4 = 0; //开启下一次捕获
        TIM8_CAPTURE_STA_CH1 = 0; //开启下一次捕获
        TIM8_CAPTURE_STA_CH2 = 0; //开启下一次捕获
        TIM2_CAPTURE_STA_CH1 = 0; //开启下一次捕获
        TIM2_CAPTURE_STA_CH2 = 0; //开启下一次捕获
        capok = 0;
        utime = 0;
        while (1)
        {
            WaitX(2);
            if (TIM4_CAPTURE_STA_CH1 & 0X80) //成功捕获到了一次上升沿
            {
                temp  = TIM4_CAPTURE_STA_CH1 & 0X3F;
                temp *= 65536;//溢出时间总和
                temp += TIM4_CAPTURE_VAL_CH1; //得到总的高电平时间
                Rc_Pwm_In[0] = temp;
                capok |= (1 << 0);
            }
            if (TIM4_CAPTURE_STA_CH2 & 0X80) //成功捕获到了一次上升沿
            {
                temp  = TIM4_CAPTURE_STA_CH2 & 0X3F;
                temp *= 65536;//溢出时间总和
                temp += TIM4_CAPTURE_VAL_CH2; //得到总的高电平时间
                Rc_Pwm_In[1] = temp;
                capok |= (1 << 1);
            }
            if (TIM3_CAPTURE_STA_CH1 & 0X80) //成功捕获到了一次上升沿
            {
                temp  = TIM3_CAPTURE_STA_CH1 & 0X3F;
                temp *= 65536;//溢出时间总和
                temp += TIM3_CAPTURE_VAL_CH1; //得到总的高电平时间
                Rc_Pwm_In[2] = temp;
                capok |= (1 << 2);
            }
            if (TIM3_CAPTURE_STA_CH2 & 0X80) //成功捕获到了一次上升沿
            {
                temp  = TIM3_CAPTURE_STA_CH2 & 0X3F;
                temp *= 65536;//溢出时间总和
                temp += TIM3_CAPTURE_VAL_CH2; //得到总的高电平时间
                Rc_Pwm_In[3] = temp;
                capok |= (1 << 3);
            }
            if (TIM3_CAPTURE_STA_CH3 & 0X80) //成功捕获到了一次上升沿
            {
                temp  = TIM3_CAPTURE_STA_CH3 & 0X3F;
                temp *= 65536;//溢出时间总和
                temp += TIM3_CAPTURE_VAL_CH3; //得到总的高电平时间
                Rc_Pwm_In[4] = temp;
                capok |= (1 << 4);
            }
            if (TIM3_CAPTURE_STA_CH4 & 0X80) //成功捕获到了一次上升沿
            {
                temp  = TIM3_CAPTURE_STA_CH4 & 0X3F;
                temp *= 65536;//溢出时间总和
                temp += TIM3_CAPTURE_VAL_CH4; //得到总的高电平时间
                Rc_Pwm_In[5] = temp;
                capok |= (1 << 5);
            }
            if (TIM8_CAPTURE_STA_CH1 & 0X80) //成功捕获到了一次上升沿
            {
                temp  = TIM1_CAPTURE_STA_CH1 & 0X3F;
                temp *= 65536;//溢出时间总和
                temp += TIM8_CAPTURE_VAL_CH1; //得到总的高电平时间
                Rc_Pwm_In[6] = temp;
                capok |= (1 << 6);
            }
            if (TIM8_CAPTURE_STA_CH2 & 0X80) //成功捕获到了一次上升沿
            {
                temp  = TIM8_CAPTURE_STA_CH2 & 0X3F;
                temp *= 65536;//溢出时间总和
                temp += TIM8_CAPTURE_VAL_CH2; //得到总的高电平时间
                Rc_Pwm_In[7] = temp;
                capok |= (1 << 7);
            }
            if (TIM2_CAPTURE_STA_CH1 & 0X80) //成功捕获到了一次上升沿
            {
                temp  = TIM2_CAPTURE_STA_CH1 & 0X3F;
                temp *= 65536;//溢出时间总和
                temp += TIM2_CAPTURE_VAL_CH1; //得到总的高电平时间
                Rc_Pwm_In[8] = temp;
                capok |= (1 << 8);
            }
            if (TIM2_CAPTURE_STA_CH2 & 0X80) //成功捕获到了一次上升沿
            {
                temp  = TIM2_CAPTURE_STA_CH2 & 0X3F;
                temp *= 65536;//溢出时间总和
                temp += TIM2_CAPTURE_VAL_CH2; //得到总的高电平时间
                Rc_Pwm_In[9] = temp;
                capok |= (1 << 9);
            }
            utime++;
            if (utime > 4 || capok == 0x3ff)
            {
                break;
            }
        }
        Rc_DataAnl(Rc_Pwm_In);//送值
        Rc_GetValue(&Rc_D);//取值
        RC_Analyse(&Rc_D, &RC_Control);//解锁
        Send_RCData = 1;
        Send_RC = 1;
        if (RC_flag_ok)
        {
            RC_flag_ok = 0;
            Data_Receive_Anl(p_RxBuffer, RC_data_cnt + 5);
        }
        Data_Exchange();
    }
    _EE
}

#include "../../GLIB/PERIPHERAL/BMP085/bmp085.h"
tg_BMP085_TYPE   bmp085;
int task_bmp085(void)
{
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
            BMP085_Printf(&bmp085);
        }
    }
    _EE
}

int loop_fast(void)//500hz
{
    extern S_INT16_XYZ Acc, Average_Acc, Gyr, Mag;
		_SS
		_LOOP_SS
    if (flag_ACC)
    {
        IMUupdate(&Gyr, &Average_Acc, &Att_Angle);//222us
        Control(&Att_Angle, &Gyr, &Rc_D, &RC_Control);//17us
		}
		LoopX(2);
		_EE
}
int main(void)
{
    SYS_INIT();
    /***总循环***/
    while (1)
    {
				RunLoop(loop_fast,0);
				
        RunTaskA(task_6050, 7);

        RunTaskA(task_ultrasonic, 1);

        RunTaskA(task_pwm_ex, 2);

        RunTaskA(task_cap_rc, 0);

        RunTaskA(task_hmc5883l, 4);

        RunTaskA(task_led, 5);

        RunTaskA(task_bmp085, 6);
    }
}
