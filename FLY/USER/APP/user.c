#include "sys_os.h"
#include "user.h"
#include "pwm.h"
//b6
//b7
//b8
//b9


struct Parameter parameter = {\
    70, \
    30, \
    10, \
    20, \
    799, \
    0, \
    799, \
    9, \
    300, \
    290, \
    0, \
    0, \
    0, \
    0, \
};


int mode2unm = 0;
/*
    u16 up;//设定上限
    u16 down;//设定下限
    u16 power;//实际功率
    u16 setpower;//设定功率
    u16 speed;//实际速度
    u16 setspeed;//实际速度
    u16 flag;//标志位
    u16 settemperature;//设定温度1
        u16 settemperature2;//设定温度2
    s16 fenshan;//风速PWM
    s16 dianzu;//电阻PWM
        u16 time_flag;
        */
extern u16 Temperature;//, humidity = 0;
struct PID spid = {\
    0, \
    90, \
    1, \
    1, \
    0, \
    0, \
    0, \
}; // PID Control Structure
struct PID fpid = {\
    0, \
    80, \
    0, \
    0, \
    0, \
    0, \
    0, \
}; // PID Control Structure

int task4()
{
    _SS
    TIM3_PWM_Init(800, 35); //9kHZ
    TIM_SetCompare1(TIM3, 0); //1000 2ms 500 1ms
    TIM_SetCompare2(TIM3, 0); //1000 2ms 500 1ms
    WaitX(600);
    WaitX(600);
    WaitX(60);
    GPIOB_Init(9, 3, PU);
    WaitX(60);
    GPIOB_Init(7, 3, PD);
    while (1)
    {
        WaitX(200);

        {
            if (parameter.flag == 10)
            {
                static int flag = 0;
                static int cha;
                static int lastT;

                cha = Temperature - parameter.settemperature;
                // > 检测温度高 需要降温
                if (flag == 1) //降温中
                {
                    if (cha * flag > 30)
                    {
                        parameter.fenshan = 799;
                        parameter.dianzu = 0;
                    }
                    else if (cha * flag > 20)
                    {
                        parameter.fenshan = 700;
                        parameter.dianzu = 0;
                    }
                    else if (cha * flag > 10)
                    {
                        parameter.fenshan = 600;
                        parameter.dianzu = 0;
                    }
                    else if (cha * flag > 3) //实际温度高
                    {
                        parameter.fenshan = 520;
                        parameter.dianzu = 0;
                    }
                    else if (cha * flag > 1) //实际温度高
                    {
                        parameter.fenshan = 500;
                        parameter.dianzu = 0;
                    }
                    else if (cha * flag > 0) //实际温度高
                    {
                        parameter.fenshan = 460;
                        parameter.dianzu = 0;
                    }
                }
                else if (flag == -1)//升温中
                {
                    if (cha * flag > 150)
                    {
                        parameter.fenshan = 0;
                        parameter.dianzu = 799;
                    }
                    else if (cha * flag > 100)
                    {
                        parameter.fenshan = 0;
                        parameter.dianzu = 300;
                    }
                    else if (cha * flag > 90)
                    {
                        parameter.fenshan = 0;
                        parameter.dianzu = 180;
                    }
                    else if (cha * flag > 30)
                    {
                        parameter.fenshan = 0;
                        parameter.dianzu = 100;
                    }
                    else if (cha * flag > 20)
                    {
                        parameter.fenshan = 0;
                        parameter.dianzu = 90;
                    }
                    else if (cha * flag > 10) //实际温度低
                    {
                        parameter.fenshan = 0;
                        parameter.dianzu = 80;
                    }
                    else if (cha * flag > 3) //实际温度低
                    {
                        parameter.fenshan = 0;
                        parameter.dianzu = 70;
                    }
                    else if (cha * flag > 2) //实际温度低
                    {
                        parameter.fenshan = 0;
                        parameter.dianzu = 60;
                    }
                    else if (cha * flag > 1) //实际温度低
                    {
                        parameter.fenshan = 0;
                        parameter.dianzu = 50;
                    }
                    else if (cha * flag > 0) //实际温度低
                    {
                        parameter.fenshan = 0;
                        parameter.dianzu = 40;
                    }
                }
                if (lastT - Temperature) //降温
                {
                    flag = 1;
                }
                else//up
                {
                    flag = -1;
                }
                lastT = Temperature;
            }
        }
        {
            if (parameter.flag == 9)
            {
                mode2unm = 0;
            }
            else if (parameter.flag == 90)
            {
                {
                    static int flag = 0;
                    static int cha;
                    static int lastT;
                    parameter.settemperature2 = FixedCurve[Curve][mode2unm] * 10;
                    cha = Temperature - parameter.settemperature2;
                    // > 检测温度高 需要降温
                    if (flag == 1) //降温中
                    {
                        if (cha * flag > 30)
                        {
                            parameter.fenshan = 799;
                            parameter.dianzu = 0;
                        }
                        else if (cha * flag > 20)
                        {
                            parameter.fenshan = 700;
                            parameter.dianzu = 0;
                        }
                        else if (cha * flag > 10)
                        {
                            parameter.fenshan = 600;
                            parameter.dianzu = 0;
                        }
                        else if (cha * flag > 3) //实际温度高
                        {
                            parameter.fenshan = 520;
                            parameter.dianzu = 0;
                        }
                        else if (cha * flag > 1) //实际温度高
                        {
                            parameter.fenshan = 500;
                            parameter.dianzu = 0;
                        }
                        else if (cha * flag > 0) //实际温度高
                        {
                            parameter.fenshan = 460;
                            parameter.dianzu = 0;
                        }
                    }
                    else if (flag == -1)//升温中
                    {
                        if (cha * flag > 150)
                        {
                            parameter.fenshan = 0;
                            parameter.dianzu = 799;
                        }
                        if (cha * flag > 100)
                        {
                            parameter.fenshan = 0;
                            parameter.dianzu = 400;
                        }
                        else if (cha * flag > 90)
                        {
                            parameter.fenshan = 0;
                            parameter.dianzu = 140;
                        }
                        else if (cha * flag > 30)
                        {
                            parameter.fenshan = 0;
                            parameter.dianzu = 130;
                        }
                        else if (cha * flag > 20)
                        {
                            parameter.fenshan = 0;
                            parameter.dianzu = 120;
                        }
                        else if (cha * flag > 10) //实际温度低
                        {
                            parameter.fenshan = 0;
                            parameter.dianzu = 110;
                        }
                        else if (cha * flag > 3) //实际温度低
                        {
                            parameter.fenshan = 0;
                            parameter.dianzu = 90;
                        }
                        else if (cha * flag > 2) //实际温度低
                        {
                            parameter.fenshan = 0;
                            parameter.dianzu = 80;
                        }
                        else if (cha * flag > 1) //实际温度低
                        {
                            parameter.fenshan = 0;
                            parameter.dianzu = 70;
                        }
                        else if (cha * flag > 0) //实际温度低
                        {
                            parameter.fenshan = 0;
                            parameter.dianzu = 60;
                        }
                    }

                    if (lastT - Temperature) //降温
                    {
                        flag = 1;
                    }
                    else//up
                    {
                        flag = -1;
                    }
                    lastT = Temperature;
                    if (parameter.time_flag == 1)
                    {

                        parameter.time_flag = 0;
                        mode2unm++;
                        if (cha > -3 && cha < 3 && mode2unm == 1)
                        {
                            mode2unm = 1;
                        }
                        else if (mode2unm == 1)
                            mode2unm = 0;
                        if (mode2unm >= 209)
                        {
                            mode2unm = 209;
                        }
                    }
                }
            }
        }
        {
            if (Temperature > parameter.up * 10)
            {
                parameter.fenshan = 799;
                parameter.dianzu = 0;
                PBout(9) = 1;
                PBout(7) = 0;
            }
            else if (Temperature < parameter.down * 10)
            {
                parameter.dianzu = 799;
                parameter.fenshan = 0;
                PBout(9) = 1;
                PBout(7) = 0;
            }
            else
            {
                PBout(9) = 0;
                PBout(7) = 1;
            }
        }
        {
            //限定幅度

            if (parameter.fenshan > 799)parameter.fenshan = 799;
            else if (parameter.fenshan < 0)parameter.fenshan = 0;
            if (parameter.dianzu > 799)parameter.dianzu = 799;
            else if (parameter.dianzu < 0)parameter.dianzu = 0;
            // if (parameter.fenshan > parameter.setpower)parameter.fenshan = parameter.setpower;
            // else if (parameter.fenshan < 0)parameter.fenshan = 0;
            // if (parameter.dianzu > parameter.setpower)parameter.dianzu = parameter.setpower;
            // else if (parameter.dianzu < 0)parameter.dianzu = 0;
        }
        parameter.dianzu = parameter.dianzu * parameter.setpower / 799.0;
        parameter.fenshan = parameter.fenshan * parameter.setspeed / 799.0;
        TIM_SetCompare1(TIM3, parameter.dianzu); //电阻
        TIM_SetCompare2(TIM3, parameter.fenshan); //1000 2ms 500 1ms
    }
    _EE
}
