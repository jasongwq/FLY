#include "rc.h"
#include "stm32_config.h"

#include "bak.h"

T_RC_Data Rc_Data;//1000~2000
T_RC_Data Rc_D;
T_Control RC_Control;

void Rc_DataAnl(u16 Rc_Pwm_In[5])
{
 #if 1==THROTTLE_INVERTED
 Rc_Data.THROTTLE        =   3000-Rc_Pwm_In[4];
 //#error
 #elif 0==THROTTLE_INVERTED
 Rc_Data.THROTTLE        =   Rc_Pwm_In[4];
 #endif
    Rc_Data.YAW             =   Rc_Pwm_In[2];
    Rc_Data.ROLL            =   Rc_Pwm_In[3];
    Rc_Data.PITCH           =   Rc_Pwm_In[1];
    Rc_Data.AUX1            =   Rc_Pwm_In[0];//左
    Rc_Data.AUX2            =   Rc_Pwm_In[5];
    Rc_Data.AUX3            =   Rc_Pwm_In[0];
    Rc_Data.AUX4            =   Rc_Pwm_In[0];
    Rc_Data.AUX5            =   Rc_Pwm_In[0];//右
    Rc_Data.AUX6            =   Rc_Pwm_In[0];
    //    rt_event_send(&Evt_Sys, EVT_RC_GET);
}
void Rc_GetValue(T_RC_Data *temp)
{
    /*if(Rc_Data.THROTTLE<2000&&Rc_Data.THROTTLE >1000)*/temp->THROTTLE  = Rc_Data.THROTTLE;
    /*if(Rc_Data.YAW     <2000&&Rc_Data.YAW      >1000)*/temp->YAW       = Rc_Data.YAW;
    /*if(Rc_Data.ROLL    <2000&&Rc_Data.ROLL     >1000)*/temp->ROLL      = Rc_Data.ROLL;
    /*if(Rc_Data.PITCH   <2000&&Rc_Data.PITCH    >1000)*/temp->PITCH     = Rc_Data.PITCH;
    /*if(Rc_Data.AUX1    <2000&&Rc_Data.AUX1     >1000)*/temp->AUX1      = Rc_Data.AUX1;
    /*if(Rc_Data.AUX2    <2000&&Rc_Data.AUX2     >1000)*/temp->AUX2      = Rc_Data.AUX2;
    /*if(Rc_Data.AUX3    <2000&&Rc_Data.AUX3     >1000)*/temp->AUX3      = Rc_Data.AUX3;
    /*if(Rc_Data.AUX4    <2000&&Rc_Data.AUX4     >1000)*/temp->AUX4      = Rc_Data.AUX4;
    /*if(Rc_Data.AUX5    <2000&&Rc_Data.AUX5     >1000)*/temp->AUX5      = Rc_Data.AUX5;
    /*if(Rc_Data.AUX6    <2000&&Rc_Data.AUX6     >1000)*/temp->AUX6      = Rc_Data.AUX6;
}

void RC_Analyse(T_RC_Data *rc_data, T_Control *ctl_data)
{
    {
        static u16 fun_cnt = 0;
        if (rc_data->THROTTLE < RC_FUN_MIN && rc_data->YAW < RC_FUN_MIN)
            if (fun_cnt < RC_FUN_CNT)
                fun_cnt++;
            else
            {
                fun_cnt = 0;
                ctl_data->ARMED = 1;
            }
        else if (rc_data->THROTTLE < RC_FUN_MIN && rc_data->YAW > RC_FUN_MAX)
            if (fun_cnt < RC_FUN_CNT)
                fun_cnt++;
            else
            {
                fun_cnt = 0;
                ctl_data->ARMED = 0;
            }
        else
            fun_cnt = 0;
    }//解锁
    {
        //开定高
        //在关闭情况下才能开
        static u16 fun_cnt2 = 0;
        if (rc_data->AUX1 > 1500 && (0 == ctl_data->ALT_ON_OFF))
        {
            if (fun_cnt2 < RC_FUN_CNT)
                fun_cnt2++;
            else
            {
                fun_cnt2 = 0;
                ctl_data->ALT_ON_OFF = 1;
                //Alt_Set = (rc_data->AUX2 - 1000) * 2;
            }
        }
        else if (rc_data->AUX1 < 1500 && ctl_data->ALT_ON_OFF)
        {
            if (fun_cnt2 < RC_FUN_CNT)
                fun_cnt2++;
            else
            {
                fun_cnt2 = 0;
                ctl_data->ALT_ON_OFF = 0;
            }
        }
        else
        {
            fun_cnt2 = 0;
        }
    }
    {
        //保存flash
        //           throrrle min
        //           yaw      max
        //           roll     min
        //           pit      min
        static u16 fun_cnt3 = 0;
        if (rc_data->THROTTLE < RC_FUN_MIN && rc_data->YAW < RC_FUN_MIN && \
                rc_data->ROLL < RC_FUN_MIN && rc_data->PITCH < RC_FUN_MIN&&\
								fun_cnt3!=(RC_FUN_CNT+10))
            if (fun_cnt3 < RC_FUN_CNT)
                fun_cnt3++;
            else
            {
                fun_cnt3 = (RC_FUN_CNT+10);
                StmFlash_Save();
            }
        else
            fun_cnt3 = 0;
    }
}
