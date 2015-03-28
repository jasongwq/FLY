/*
void Ex_Anl(void)
{
    u16 temp = (u16) * ex_on_off;
    Ex_ON_OFF.Status = Read_Bit(temp, 0);//1
    Ex_ON_OFF.Senser = Read_Bit(temp, 1);//2
    Ex_ON_OFF.DataF1 = Read_Bit(temp, 2);//4
    Ex_ON_OFF.DataF2 = Read_Bit(temp, 3);//8
    Ex_ON_OFF.DataF3 = Read_Bit(temp, 4);//16
    Ex_ON_OFF.DataF4 = Read_Bit(temp, 5);//32
    Ex_ON_OFF.RCData = Read_Bit(temp, 6);//64
    Ex_ON_OFF.MotoPwm = Read_Bit(temp, 7);//128
}
*/
#include "data_transfer.h"
#include "sys.h"
#include "mpu6050.h"
#include "usr_usart.h"
#include "control.h"
#include "Balance.h"
#include "IMU.h"
#include "rc.h"
#include "bak.h"
#include "pwm.h"
//#include "bmp085.h"
#define DATA_TRANSFER_USE_USART

#define PID_ROL_P_MULTIPLYING   (100 )
#define PID_ROL_I_MULTIPLYING   (1000)
#define PID_ROL_D_MULTIPLYING   (100 )
#define PID_PIT_P_MULTIPLYING   (100 )
#define PID_PIT_I_MULTIPLYING   (1000)
#define PID_PIT_D_MULTIPLYING   (100 )
#define PID_YAW_P_MULTIPLYING   (100 )
#define PID_YAW_I_MULTIPLYING   (100 )
#define PID_YAW_D_MULTIPLYING   (100 )

#define PID_ALT_P_MULTIPLYING   (1000)
#define PID_ALT_I_MULTIPLYING   (1000)
#define PID_ALT_D_MULTIPLYING   (1000)
#define PID_POS_P_MULTIPLYING   (1000)
#define PID_POS_I_MULTIPLYING   (1000)
#define PID_POS_D_MULTIPLYING   (1000)
#define PID_PID_1_P_MULTIPLYING (100 )
#define PID_PID_1_I_MULTIPLYING (100 )
#define PID_PID_1_D_MULTIPLYING (100 )

float *ex_on_off = &PID_PID_2.P;
#define PID_PID_2_P_MULTIPLYING (1  )
#define PID_PID_2_I_MULTIPLYING (1  )
#define PID_PID_2_D_MULTIPLYING (1  )
float *KALMAN_Q = &PID_PID_3.P;
float *KALMAN_R = &PID_PID_3.I;
float *KALMAN_P0 = &PID_PID_3.D;
#define PID_PID_3_P_MULTIPLYING (10000  )
#define PID_PID_3_I_MULTIPLYING (100  )
#define PID_PID_3_D_MULTIPLYING (1  )
#define PID_PID_4_P_MULTIPLYING (1  )
#define PID_PID_4_I_MULTIPLYING (1  )
#define PID_PID_4_D_MULTIPLYING (1  )

#define PID_PID_5_P_MULTIPLYING (1  )
#define PID_PID_5_I_MULTIPLYING (1  )
#define PID_PID_5_D_MULTIPLYING (1  )
#define PID_PID_6_P_MULTIPLYING (1  )
#define PID_PID_6_I_MULTIPLYING (1  )
#define PID_PID_6_D_MULTIPLYING (1  )
#define PID_PID_7_P_MULTIPLYING (1  )
#define PID_PID_7_I_MULTIPLYING (1  )
#define PID_PID_7_D_MULTIPLYING (1  )

#define PID_PID_8_P_MULTIPLYING  (1  )
#define PID_PID_8_I_MULTIPLYING  (1  )
#define PID_PID_8_D_MULTIPLYING  (1  )
#define PID_PID_9_P_MULTIPLYING  (1  )
#define PID_PID_9_I_MULTIPLYING  (1  )
#define PID_PID_9_D_MULTIPLYING  (1  )
#define PID_PID_10_P_MULTIPLYING (1  )
#define PID_PID_10_I_MULTIPLYING (1  )
#define PID_PID_10_D_MULTIPLYING (1  )

#define PID_PID_11_P_MULTIPLYING (1  )
#define PID_PID_11_I_MULTIPLYING (1  )
#define PID_PID_11_D_MULTIPLYING (1  )
float *yaw_just  = &PID_PID_12.P;
float *rol_just  = &PID_PID_12.I;
float *pit_just  = &PID_PID_12.D;
#define PID_PID_12_P_MULTIPLYING (100)
#define PID_PID_12_I_MULTIPLYING (100)
#define PID_PID_12_D_MULTIPLYING (100)

void Ex_Init(void)
{
    *KALMAN_Q  = 0.001;
    *KALMAN_R  = 20.0000;
    *KALMAN_P0 = 2210.0000;
    PID_ROL.P = 4;
    PID_PIT.P = 4;
    PID_YAW.P = 4;
    PID_ROL.I = 0.002;
    PID_PIT.I = 0.002;
    PID_YAW.I = 0.005;
    PID_ROL.D = 0.13;
    PID_PIT.D = 0.13;
    PID_YAW.D = 0.30;
    PID_ALT.P = 0.1;
    PID_ALT.I = 0.01;
    PID_ALT.D = 0.2;

    *ex_on_off = 1;
    *yaw_just = 10;
    *rol_just = 8;
    *pit_just = 10;
}
void Ex_Anl(void)
{
    u16 temp = (u16) * ex_on_off;
    Ex_ON_OFF.Status = Read_Bit(temp, 0);//1
    Ex_ON_OFF.Senser = Read_Bit(temp, 1);//2
    Ex_ON_OFF.DataF1 = Read_Bit(temp, 2);//4
    Ex_ON_OFF.DataF2 = Read_Bit(temp, 3);//8
    Ex_ON_OFF.DataF3 = Read_Bit(temp, 4);//16
    Ex_ON_OFF.DataF4 = Read_Bit(temp, 5);//32
    Ex_ON_OFF.RCData = Read_Bit(temp, 6);//64
    Ex_ON_OFF.MotoPwm = Read_Bit(temp, 7);//128
}
struct DATA_TRANSFER_SWITCH Ex_ON_OFF, Send;

S_INT16_XYZ Acc, Average_Acc, Average2_Acc, Average3_Acc, Average1_Gyr, Average2_Gyr, Gyr, Mag;
void Data_Send_Check(u16 check);
void Data_Receive_Anl(u8 *data_buf, u8 num)
{
    vs16 rc_value_temp;
    u8 sum = 0;
    //Sys_sPrintf(Printf_USART, data_buf, num);
    for (u8 i = 0; i < (num - 1); i++)
        sum += *(data_buf + i);
    if (!(sum == *(data_buf + num - 1)))       return; //sum
    if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF))     return; //
    /////////////////////////////////////////////////////////////////////////////////////
    if (*(data_buf + 2) == 0X01)
    {
        if (*(data_buf + 4) == 0X01)
            MPU6050_CalOff_Acc();
        if (*(data_buf + 4) == 0X02)
            MPU6050_CalOff_Gyr();
        if (*(data_buf + 4) == 0X03)
        {
            MPU6050_CalOff_Acc();
            MPU6050_CalOff_Gyr();
        }
        if (*(data_buf + 4) == 0X04)
            ;//Cal_Compass();
        if (*(data_buf + 4) == 0X05)
            ;//MS5611_CalOffset();
    }
    if (*(data_buf + 2) == 0X02)
    {
        if (*(data_buf + 4) == 0X01)
        {
            Send.PID1 = 1;
            Send.PID2 = 1;
            Send.PID3 = 1;
            Send.PID4 = 1;
            Send.PID5 = 1;
            Send.PID6 = 1;
        }
        if (*(data_buf + 4) == 0X02)
            Send.Offset = 1;
    }
    if (*(data_buf + 2) == 0X10)                        //PID1
    {
        PID_ROL.P = (float)((vs16)(*(data_buf + 4 ) << 8) | *(data_buf + 5 )) / PID_ROL_P_MULTIPLYING;
        PID_ROL.I = (float)((vs16)(*(data_buf + 6 ) << 8) | *(data_buf + 7 )) / PID_ROL_I_MULTIPLYING;
        PID_ROL.D = (float)((vs16)(*(data_buf + 8 ) << 8) | *(data_buf + 9 )) / PID_ROL_D_MULTIPLYING;
        PID_PIT.P = (float)((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11)) / PID_PIT_P_MULTIPLYING;
        PID_PIT.I = (float)((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13)) / PID_PIT_I_MULTIPLYING;
        PID_PIT.D = (float)((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15)) / PID_PIT_D_MULTIPLYING;
        PID_YAW.P = (float)((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17)) / PID_YAW_P_MULTIPLYING;
        PID_YAW.I = (float)((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19)) / PID_YAW_I_MULTIPLYING;
        PID_YAW.D = (float)((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21)) / PID_YAW_D_MULTIPLYING;
        Data_Send_Check(sum);//
        Data_Save(1);
        Send.PID1 = 1;
    }
    if (*(data_buf + 2) == 0X11)                        //PID2
    {
        PID_ALT.P   = (float)((vs16)(*(data_buf + 4 ) << 8) | *(data_buf + 5 )) / PID_ALT_P_MULTIPLYING  ;
        PID_ALT.I   = (float)((vs16)(*(data_buf + 6 ) << 8) | *(data_buf + 7 )) / PID_ALT_I_MULTIPLYING  ;
        PID_ALT.D   = (float)((vs16)(*(data_buf + 8 ) << 8) | *(data_buf + 9 )) / PID_ALT_D_MULTIPLYING  ;
        PID_POS.P   = (float)((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11)) / PID_POS_P_MULTIPLYING  ;
        PID_POS.I   = (float)((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13)) / PID_POS_I_MULTIPLYING  ;
        PID_POS.D   = (float)((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15)) / PID_POS_D_MULTIPLYING  ;
        PID_PID_1.P = (float)((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17)) / PID_PID_1_P_MULTIPLYING;
        PID_PID_1.I = (float)((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19)) / PID_PID_1_I_MULTIPLYING;
        PID_PID_1.D = (float)((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21)) / PID_PID_1_D_MULTIPLYING;
        Send.PID2 = 1;
        Data_Send_Check(sum);
    }
    if (*(data_buf + 2) == 0X12)                        //PID3
    {
        PID_PID_2.P = (float)((vs16)(*(data_buf + 4 ) << 8) | *(data_buf + 5 )) / PID_PID_2_P_MULTIPLYING;
        PID_PID_2.I = (float)((vs16)(*(data_buf + 6 ) << 8) | *(data_buf + 7 )) / PID_PID_2_I_MULTIPLYING;
        PID_PID_2.D = (float)((vs16)(*(data_buf + 8 ) << 8) | *(data_buf + 9 )) / PID_PID_2_D_MULTIPLYING;
        PID_PID_3.P = (float)((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11)) / PID_PID_3_P_MULTIPLYING;
        PID_PID_3.I = (float)((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13)) / PID_PID_3_I_MULTIPLYING;
        PID_PID_3.D = (float)((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15)) / PID_PID_3_D_MULTIPLYING;
        PID_PID_4.P = (float)((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17)) / PID_PID_4_P_MULTIPLYING;
        PID_PID_4.I = (float)((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19)) / PID_PID_4_I_MULTIPLYING;
        PID_PID_4.D = (float)((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21)) / PID_PID_4_D_MULTIPLYING;
        Send.PID3 = 1;
        Data_Send_Check(sum);
        Data_Save(1);
    }
    if (*(data_buf + 2) == 0X13)                        //PID4
    {
        PID_PID_5.P = (float)((vs16)(*(data_buf + 4 ) << 8) | *(data_buf + 5 )) / PID_PID_5_P_MULTIPLYING;
        PID_PID_5.I = (float)((vs16)(*(data_buf + 6 ) << 8) | *(data_buf + 7 )) / PID_PID_5_I_MULTIPLYING;
        PID_PID_5.D = (float)((vs16)(*(data_buf + 8 ) << 8) | *(data_buf + 9 )) / PID_PID_5_D_MULTIPLYING;
        PID_PID_6.P = (float)((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11)) / PID_PID_6_P_MULTIPLYING;
        PID_PID_6.I = (float)((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13)) / PID_PID_6_I_MULTIPLYING;
        PID_PID_6.D = (float)((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15)) / PID_PID_6_D_MULTIPLYING;
        PID_PID_7.P = (float)((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17)) / PID_PID_7_P_MULTIPLYING;
        PID_PID_7.I = (float)((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19)) / PID_PID_7_I_MULTIPLYING;
        PID_PID_7.D = (float)((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21)) / PID_PID_7_D_MULTIPLYING;
        Data_Send_Check(sum);
    }
    if (*(data_buf + 2) == 0X14)                        //PID5
    {
        PID_PID_8.P =  (float)((vs16)(*(data_buf + 4) << 8 ) | *(data_buf + 5 )) / PID_PID_8_P_MULTIPLYING ;
        PID_PID_8.I =  (float)((vs16)(*(data_buf + 6) << 8 ) | *(data_buf + 7 )) / PID_PID_8_I_MULTIPLYING ;
        PID_PID_8.D =  (float)((vs16)(*(data_buf + 8) << 8 ) | *(data_buf + 9 )) / PID_PID_8_D_MULTIPLYING ;
        PID_PID_9.P =  (float)((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11)) / PID_PID_9_P_MULTIPLYING ;
        PID_PID_9.I =  (float)((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13)) / PID_PID_9_I_MULTIPLYING ;
        PID_PID_9.D =  (float)((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15)) / PID_PID_9_D_MULTIPLYING ;
        PID_PID_10.P = (float)((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17)) / PID_PID_10_P_MULTIPLYING;
        PID_PID_10.I = (float)((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19)) / PID_PID_10_I_MULTIPLYING;
        PID_PID_10.D = (float)((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21)) / PID_PID_10_D_MULTIPLYING;
        Data_Send_Check(sum);
    }
    if (*(data_buf + 2) == 0X15)                        //PID6
    {
        PID_PID_11.P = (float)((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5))   / PID_PID_11_P_MULTIPLYING;
        PID_PID_11.I = (float)((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7))   / PID_PID_11_I_MULTIPLYING;
        PID_PID_11.D = (float)((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9))   / PID_PID_11_D_MULTIPLYING;
        PID_PID_12.P = (float)((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11)) / PID_PID_12_P_MULTIPLYING;
        PID_PID_12.I = (float)((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13)) / PID_PID_12_I_MULTIPLYING;
        PID_PID_12.D = (float)((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15)) / PID_PID_12_D_MULTIPLYING;
        Data_Send_Check(sum);
    }
    if (*(data_buf + 2) == 0X16)                        //OFFSET
    {
        AngleOffset_Rol = (float)((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5)) / 1000;
        AngleOffset_Pit = (float)((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7)) / 1000;
        //          Data_Save();
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////
    if (*(data_buf + 2) == 0x03)                        //
    {

    }
    Ex_Anl();
}
u8 data_to_send[120];
void Data_Send_Check(u16 check)
{
    data_to_send[0] = 0xAA;
    data_to_send[1] = 0xAA;
    data_to_send[2] = 0xF0;
    data_to_send[3] = 3;
    data_to_send[4] = 0xBA;
    data_to_send[5] = BYTE1(check);
    data_to_send[6] = BYTE0(check);
    u8 sum = 0;
    for (u8 i = 0; i < 7; i++)
        sum += data_to_send[i];
    data_to_send[7] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, 8);
#else
    NRF_TxPacket(data_to_send, 8);
#endif
}
extern u16 Alt_ultrasonic;
extern s32 Alt_bmp;

void Data_Send_Status(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0;
    vs16 _temp;
    _temp = (int)(Att_Angle.rol * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(Att_Angle.pit * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(Att_Angle.yaw * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = Alt_ultrasonic;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    vs32 _temp2 = Alt_bmp;
    data_to_send[_cnt++] = BYTE3(_temp2);
    data_to_send[_cnt++] = BYTE2(_temp2);
    data_to_send[_cnt++] = BYTE1(_temp2);
    data_to_send[_cnt++] = BYTE0(_temp2);
    if (0 == RC_Control.ARMED)
        data_to_send[_cnt++] = 0xA0;
    else if (1 == RC_Control.ARMED)
        data_to_send[_cnt++] = 0xA1;
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
extern float rol_i;
//extern s32 Alt_Error;

void Data_Send_Senser(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0;
    vs16 _temp;
#define EX_ACC Acc
    data_to_send[_cnt++] = BYTE1(EX_ACC.x);
    data_to_send[_cnt++] = BYTE0(EX_ACC.x);
    data_to_send[_cnt++] = BYTE1(EX_ACC.y);
    data_to_send[_cnt++] = BYTE0(EX_ACC.y);
    data_to_send[_cnt++] = BYTE1(EX_ACC.z);
    data_to_send[_cnt++] = BYTE0(EX_ACC.z);
    data_to_send[_cnt++] = BYTE1(Gyr.x);
    data_to_send[_cnt++] = BYTE0(Gyr.x);
    data_to_send[_cnt++] = BYTE1(Gyr.y);
    data_to_send[_cnt++] = BYTE0(Gyr.y);
    data_to_send[_cnt++] = BYTE1(Gyr.z);
    data_to_send[_cnt++] = BYTE0(Gyr.z);
    data_to_send[_cnt++] = BYTE1(Mag.x);
    data_to_send[_cnt++] = BYTE0(Mag.x);
    data_to_send[_cnt++] = BYTE1(Mag.y);
    data_to_send[_cnt++] = BYTE0(Mag.y);
    data_to_send[_cnt++] = BYTE1(Mag.z);
    data_to_send[_cnt++] = BYTE0(Mag.z);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
void Data_Send_OFFSET(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x16;
    data_to_send[_cnt++] = 0;
    vs16 _temp = AngleOffset_Rol * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = AngleOffset_Pit * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
void Data_Send_PID1(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x10;
    data_to_send[_cnt++] = 0;
    vs16 _temp;
    _temp = PID_ROL.P * PID_ROL_P_MULTIPLYING; data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_ROL.I * PID_ROL_I_MULTIPLYING; data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_ROL.D * PID_ROL_D_MULTIPLYING; data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_PIT.P * PID_PIT_P_MULTIPLYING; data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_PIT.I * PID_PIT_I_MULTIPLYING; data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_PIT.D * PID_PIT_D_MULTIPLYING; data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_YAW.P * PID_YAW_P_MULTIPLYING; data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_YAW.I * PID_YAW_I_MULTIPLYING; data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_YAW.D * PID_YAW_D_MULTIPLYING; data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}


void Data_Send_PID2(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x11;
    data_to_send[_cnt++] = 0;
    vs16 _temp;
    _temp = (vs16)(PID_ALT.P   * PID_ALT_P_MULTIPLYING  ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_ALT.I   * PID_ALT_I_MULTIPLYING  ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_ALT.D   * PID_ALT_D_MULTIPLYING  ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_POS.P   * PID_POS_P_MULTIPLYING  ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_POS.I   * PID_POS_I_MULTIPLYING  ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_POS.D   * PID_POS_D_MULTIPLYING  ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_1.P * PID_PID_1_P_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_1.I * PID_PID_1_I_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_1.D * PID_PID_1_D_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}

//¡ö8£º·¢ËÍPID3
void Data_Send_PID3(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x12;
    data_to_send[_cnt++] = 0;
    vs16 _temp;
    _temp = (vs16)(PID_PID_2.P * PID_PID_2_P_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_2.I * PID_PID_2_I_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_2.D * PID_PID_2_D_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_3.P * PID_PID_3_P_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_3.I * PID_PID_3_I_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_3.D * PID_PID_3_D_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_4.P * PID_PID_4_P_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_4.I * PID_PID_4_I_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_4.D * PID_PID_4_D_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}

//¡ö8£º·¢ËÍPID4
void Data_Send_PID4(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x13;
    data_to_send[_cnt++] = 0;
    vs16 _temp;
    _temp = (vs16)(PID_PID_5.P * PID_PID_5_P_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_5.I * PID_PID_5_I_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_5.D * PID_PID_5_D_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_6.P * PID_PID_6_P_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_6.I * PID_PID_6_I_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_6.D * PID_PID_6_D_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_7.P * PID_PID_7_P_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_7.I * PID_PID_7_I_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_7.D * PID_PID_7_D_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
void Data_Send_PID5(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x14;
    data_to_send[_cnt++] = 0;
    vs16 _temp;
    _temp = (vs16)(PID_PID_8.P  * PID_PID_8_P_MULTIPLYING ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_8.I  * PID_PID_8_I_MULTIPLYING ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_8.D  * PID_PID_8_D_MULTIPLYING ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_9.P  * PID_PID_9_P_MULTIPLYING ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_9.I  * PID_PID_9_I_MULTIPLYING ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_9.D  * PID_PID_9_D_MULTIPLYING ); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_10.P * PID_PID_10_P_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_10.I * PID_PID_10_I_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_10.D * PID_PID_10_D_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
void Data_Send_PID6(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x15;
    data_to_send[_cnt++] = 0;
    vs16 _temp;
    _temp = (vs16)(PID_PID_11.P * PID_PID_11_P_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_11.I * PID_PID_11_I_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_11.D * PID_PID_11_D_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_12.P * PID_PID_12_P_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_12.I * PID_PID_12_I_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (vs16)(PID_PID_12.D * PID_PID_12_D_MULTIPLYING); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
void Data_Send_RCData(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x03;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = BYTE1(Rc_D.THROTTLE);
    data_to_send[_cnt++] = BYTE0(Rc_D.THROTTLE);
    data_to_send[_cnt++] = BYTE1(Rc_D.YAW);
    data_to_send[_cnt++] = BYTE0(Rc_D.YAW);
    data_to_send[_cnt++] = BYTE1(Rc_D.ROLL);
    data_to_send[_cnt++] = BYTE0(Rc_D.ROLL);
    data_to_send[_cnt++] = BYTE1(Rc_D.PITCH);
    data_to_send[_cnt++] = BYTE0(Rc_D.PITCH);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX1);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX1);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX2);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX2);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX3);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX3);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX4);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX4);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX5);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX5);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX6);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX6);
    data_to_send[_cnt++] = BYTE3(Alt_ultrasonic);
    data_to_send[_cnt++] = BYTE2(Alt_ultrasonic);
    data_to_send[_cnt++] = BYTE1(Alt_ultrasonic);
    data_to_send[_cnt++] = BYTE0(Alt_ultrasonic);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
void Data_Send_RC(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAF;
    data_to_send[_cnt++] = 0x18;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = BYTE1(Rc_D.THROTTLE);
    data_to_send[_cnt++] = BYTE0(Rc_D.THROTTLE);
    data_to_send[_cnt++] = BYTE1(Rc_D.YAW);
    data_to_send[_cnt++] = BYTE0(Rc_D.YAW);
    data_to_send[_cnt++] = BYTE1(Rc_D.ROLL);
    data_to_send[_cnt++] = BYTE0(Rc_D.ROLL);
    data_to_send[_cnt++] = BYTE1(Rc_D.PITCH);
    data_to_send[_cnt++] = BYTE0(Rc_D.PITCH);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX1);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX1);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX2);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX2);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX3);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX3);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX4);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX4);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX5);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX5);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX6);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX6);
    data_to_send[_cnt++] = BYTE1(Alt_ultrasonic);
    data_to_send[_cnt++] = BYTE0(Alt_ultrasonic);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}

void Data_Send_MotoPWM(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x06;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = BYTE1(MOTO1_PWM);
    data_to_send[_cnt++] = BYTE0(MOTO1_PWM);
    data_to_send[_cnt++] = BYTE1(MOTO2_PWM);
    data_to_send[_cnt++] = BYTE0(MOTO2_PWM);
    data_to_send[_cnt++] = BYTE1(MOTO3_PWM);
    data_to_send[_cnt++] = BYTE0(MOTO3_PWM);
    data_to_send[_cnt++] = BYTE1(MOTO4_PWM);
    data_to_send[_cnt++] = BYTE0(MOTO4_PWM);
    data_to_send[_cnt++] = BYTE1(MOTO1_PWM);
    data_to_send[_cnt++] = BYTE0(MOTO1_PWM);
    data_to_send[_cnt++] = BYTE1(MOTO1_PWM);
    data_to_send[_cnt++] = BYTE0(MOTO1_PWM);
    data_to_send[_cnt++] = BYTE1(MOTO1_PWM);
    data_to_send[_cnt++] = BYTE0(MOTO1_PWM);
    data_to_send[_cnt++] = BYTE1(MOTO1_PWM);
    data_to_send[_cnt++] = BYTE0(MOTO1_PWM);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
void Data_Send_F1(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xF1;
    data_to_send[_cnt++] = 0;
    //        __IO float _tempf;
    //        _tempf = position_x;data_to_send[_cnt++] = BYTE3(_tempf);data_to_send[_cnt++] = BYTE2(_tempf);data_to_send[_cnt++] = BYTE1(_tempf);data_to_send[_cnt++] = BYTE0(_tempf);
    //        _tempf = position_y;data_to_send[_cnt++] = BYTE3(_tempf);data_to_send[_cnt++] = BYTE2(_tempf);data_to_send[_cnt++] = BYTE1(_tempf);data_to_send[_cnt++] = BYTE0(_tempf);
    //        _tempf = position_z;data_to_send[_cnt++] = BYTE3(_tempf);data_to_send[_cnt++] = BYTE2(_tempf);data_to_send[_cnt++] = BYTE1(_tempf);data_to_send[_cnt++] = BYTE0(_tempf);
    vs16 _temp16;
    _temp16 = (vs16)(Acc.x  * PID_PID_8_P_MULTIPLYING ); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    _temp16 = (vs16)(Average_Acc.x  * PID_PID_8_I_MULTIPLYING ); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    _temp16 = (vs16)(Average2_Acc.x  * PID_PID_8_P_MULTIPLYING ); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    //    _temp16 = (vs16)(Average3_Acc.x  * PID_PID_8_I_MULTIPLYING ); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);

    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
extern s32 Alt_bmp1;
extern s32 Alt_bmp2;
//8
void Data_Send_F2(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xF2;
    data_to_send[_cnt++] = 0;
    vs16 _temp16;
    vs32 _temp32;

    //_temp16 = (vs16)(PID_PIT.OUT  ); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    _temp32 = (vs32)(Alt_bmp); data_to_send[_cnt++] = BYTE3(_temp32); data_to_send[_cnt++] = BYTE2(_temp32); data_to_send[_cnt++] = BYTE1(_temp32); data_to_send[_cnt++] = BYTE0(_temp32);
    _temp32 = (vs32)(Alt_bmp1); data_to_send[_cnt++] = BYTE3(_temp32); data_to_send[_cnt++] = BYTE2(_temp32); data_to_send[_cnt++] = BYTE1(_temp32); data_to_send[_cnt++] = BYTE0(_temp32);
    _temp32 = (vs32)(Alt_bmp2); data_to_send[_cnt++] = BYTE3(_temp32); data_to_send[_cnt++] = BYTE2(_temp32); data_to_send[_cnt++] = BYTE1(_temp32); data_to_send[_cnt++] = BYTE0(_temp32);

    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
//16
void Data_Send_F3(void)
{
    extern u32 Throttle_OUT;
    // T_float_angle angle;
    extern s16 Alt_Error;
    extern u16 Alt_ultrasonic1t;
    extern u16 Alt_ultrasonic2t;
    extern u32 Throttle_OUT;
    extern u32 Throttle_IN;
    //extern u32 Throttle_OUT;

    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xF3;
    data_to_send[_cnt++] = 0;
    vs16 _temp16;
    _temp16 = (vs16)(Alt_ultrasonic1t); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    _temp16 = (vs16)(Alt_ultrasonic2t ); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    _temp16 = (vs16)(Throttle_OUT ); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    _temp16 = (vs16)(Throttle_IN); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    _temp16 = (vs16)(Alt_ultrasonic); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    _temp16 = (vs16)(PID_ALT.OUT ); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    _temp16 = (vs16)(Alt_Error   ); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    _temp16 = (vs16)(Balance_Throttle   ); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);
    //_temp16 = (vs16)(angle.yaw  * PID_PID_8_P_MULTIPLYING ); data_to_send[_cnt++] = BYTE1(_temp16); data_to_send[_cnt++] = BYTE0(_temp16);

    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
//32
void Data_Send_F4(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xF4;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = BYTE1(Rc_D.THROTTLE);
    data_to_send[_cnt++] = BYTE0(Rc_D.THROTTLE);
    data_to_send[_cnt++] = BYTE1(Rc_D.YAW);
    data_to_send[_cnt++] = BYTE0(Rc_D.YAW);
    data_to_send[_cnt++] = BYTE1(Rc_D.ROLL);
    data_to_send[_cnt++] = BYTE0(Rc_D.ROLL);
    data_to_send[_cnt++] = BYTE1(Rc_D.PITCH);
    data_to_send[_cnt++] = BYTE0(Rc_D.PITCH);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX1);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX1);
    data_to_send[_cnt++] = BYTE1(Rc_D.AUX2);
    data_to_send[_cnt++] = BYTE0(Rc_D.AUX2);
    //    data_to_send[_cnt++] = BYTE1(Rc_D.AUX3);
    //    data_to_send[_cnt++] = BYTE0(Rc_D.AUX3);
    //    data_to_send[_cnt++] = BYTE1(Rc_D.AUX4);
    //    data_to_send[_cnt++] = BYTE0(Rc_D.AUX4);
    //    data_to_send[_cnt++] = BYTE1(Rc_D.AUX5);
    //    data_to_send[_cnt++] = BYTE0(Rc_D.AUX5);
    //    data_to_send[_cnt++] = BYTE1(Rc_D.AUX6);
    //    data_to_send[_cnt++] = BYTE0(Rc_D.AUX6);
    //    data_to_send[_cnt++] = BYTE1(Alt_ultrasonic);
    //    data_to_send[_cnt++] = BYTE0(Alt_ultrasonic);
    data_to_send[3] = _cnt - 4;
    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(Printf_USART, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
void Data_Exchange(void)
{
#ifdef DATA_TRANSFER_USE_SPI_NRF
    Nrf_Check_Event();
    u8 sta = Nrf_Get_FIFOSta();
    if ((sta & (1 << 4)) == 0)
        return;
#endif
    if (Send.DataF1)
    {
        Send.DataF1 = 0;
        Data_Send_F1();
    }
    if (Send.DataF2)
    {
        Send.DataF2 = 0;
        Data_Send_F2();
    }
    if (Send.DataF3)
    {
        Send.DataF3 = 0;
        Data_Send_F3();
    }
    if (Send.RCData && Send.DataF4)
    {
        Send.DataF4 = 0;
        Data_Send_F4();
    }
    if (Send.Status)
    {
        Send.Status = 0;
        Data_Send_Status();
    }
    else if (Send.GpsData)
    {
        Send.GpsData = 0;
        //Data_Send_GpsData();
    }
    if (Send.Senser)
    {
        Send.Senser = 0;
        Data_Send_Senser();
    }
    if (Send.PID1)
    {
        Send.PID1 = 0;
        Data_Send_PID1();
    }
    else if (Send.PID2)
    {
        Send.PID2 = 0;
        Data_Send_PID2();
    }
    else if (Send.PID3)
    {
        Send.PID3 = 0;
        Data_Send_PID3();
    }
    else if (Send.PID4)
    {
        Send.PID4 = 0;
        Data_Send_PID4();
    }
    else if (Send.PID5)
    {
        Send.PID5 = 0;
        Data_Send_PID5();
    }
    else if (Send.PID6)
    {
        Send.PID6 = 0;
        Data_Send_PID6();
    }
    if (Send.RCData && Ex_ON_OFF.RCData)
    {
        Send.RCData = 0;
        Data_Send_RCData();
    }
    if (Send.Offset)
    {
        Send.Offset = 0;
        Data_Send_OFFSET();
    }
    if (Send.MotoPwm)
    {
        Send.MotoPwm = 0;
        Data_Send_MotoPWM();
    }
}
