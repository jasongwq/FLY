#include "sys.h"
#include "mpu6050.h"
#include "usr_usart.h"
#include "control.h"
#include "IMU.h"
#include "data_transfer.h"
#include "rc.h"
#include "bak.h"
#include "pwm.h"
extern int32_t MOTO1_PWM, MOTO2_PWM, MOTO3_PWM, MOTO4_PWM;


#define DATA_TRANSFER_USE_USART

u8 Data_Check, Send_Status, Send_Senser, Send_RCData, Send_RC, Send_GpsData, Send_Offset, Send_PID1, Send_PID2, Send_PID3, Send_MotoPwm;


S_INT16_XYZ Acc, Average_Acc, Gyr, Mag;
void Data_Receive_Anl(u8 *data_buf, u8 num)
{
    vs16 rc_value_temp;
    u8 sum = 0;
    Sys_sPrintf(USART1, data_buf, num);
    for (u8 i = 0; i < (num - 1); i++)
        sum += *(data_buf + i);
    if (!(sum == *(data_buf + num - 1)))       return; //sum
    if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF))     return; //
    /////////////////////////////////////////////////////////////////////////////////////
    //    if (*(data_buf + 2) == 0X01)
    //    {
    //        if (*(data_buf + 4) == 0X01)
    //            MPU6050_CalOff_Acc();
    //        if (*(data_buf + 4) == 0X02)
    //            MPU6050_CalOff_Gyr();
    //        if (*(data_buf + 4) == 0X03)
    //        {
    //            MPU6050_CalOff_Acc();
    //            MPU6050_CalOff_Gyr();
    //        }
    //        if (*(data_buf + 4) == 0X04)
    //            ;//Cal_Compass();
    //        if (*(data_buf + 4) == 0X05)
    //            ;//MS5611_CalOffset();
    //    }
    //    if (*(data_buf + 2) == 0X02)
    //    {
    //        if (*(data_buf + 4) == 0X01)
    //        {
    //            Send_PID1 = 1;
    //            Send_PID2 = 1;
    //            Send_PID3 = 1;
    //        }
    //        if (*(data_buf + 4) == 0X02)
    //            Send_Offset = 1;
    //    }
    //    if (*(data_buf + 2) == 0X10)                        //PID1
    //    {
    //        PID_ROL.P = (float)((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5)) / 100;
    //        PID_ROL.I = (float)((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7)) / 1000;
    //        PID_ROL.D = (float)((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9)) / 100;
    //        PID_PIT.P = (float)((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11)) / 100;
    //        PID_PIT.I = (float)((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13)) / 1000;
    //        PID_PIT.D = (float)((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15)) / 100;
    //        PID_YAW.P = (float)((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17)) / 100;
    //        PID_YAW.I = (float)((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19)) / 100;
    //        PID_YAW.D = (float)((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21)) / 100;
    //        Data_Save(1);
    //        Send_PID1 = 1;
    //    }
    //    if (*(data_buf + 2) == 0X11)                        //PID2
    //    {
    //        PID_ALT.P = (float)((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5)) / 1000;
    //        PID_ALT.I = (float)((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7)) / 100;
    //        PID_ALT.D = (float)((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9)) / 100;
    //        PID_POS.P = (float)((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11)) / 100;
    //        PID_POS.I = (float)((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13)) / 100;
    //        PID_POS.D = (float)((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15)) / 100;
    //        PID_PID_1.P = (float)((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17)) / 100;
    //        PID_PID_1.I = (float)((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19)) / 100;
    //        PID_PID_1.D = (float)((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21)) / 100;
    //        Send_PID2 = 1;
    //    }
    //    if (*(data_buf + 2) == 0X12)                        //PID3
    //    {
    //        PID_PID_2.P = (float)((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5)) / 100;
    //        PID_PID_2.I = (float)((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7)) / 100;
    //        PID_PID_2.D = (float)((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9)) / 100;
    //        Send_PID3 = 1;
    //        Data_Save(1);
    //    }
    //    if (*(data_buf + 2) == 0X16)                        //OFFSET
    //    {
    //        //          AngleOffset_Rol = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
    //        //          AngleOffset_Pit = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
    //        //          Data_Save();
    //    }

    //    /////////////////////////////////////////////////////////////////////////////////////////////////
    //    if (*(data_buf + 2) == 0x03)                        //
    //    {

    //    }
}
u8 data_to_send[120];

extern u16 Alt_ultrasonic;
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
    //_temp = (int)(Q_ANGLE.YAW*100);
    //_temp = (int)(Mag_Heading*100);
    _temp = (int)(Att_Angle.yaw * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    vs32 _temp2 = Alt_ultrasonic;
    data_to_send[_cnt++] = BYTE3(_temp2);
    data_to_send[_cnt++] = BYTE2(_temp2);
    data_to_send[_cnt++] = BYTE1(_temp2);
    data_to_send[_cnt++] = BYTE0(_temp2);


    if (1)           data_to_send[_cnt++] = 0xA0;
    else if (1)      data_to_send[_cnt++] = 0xA1;

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(USART1, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
extern float rol_i;
extern s32 Alt_Error;

void Data_Send_Senser(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0;
		    vs16 _temp;
    _temp = (int)(Acc.x / -100);
		data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
		_temp = (int)(Acc.y / -100);
		data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
		
//    data_to_send[_cnt++] = BYTE1(Acc.x);
//    data_to_send[_cnt++] = BYTE0(Acc.x);
//    data_to_send[_cnt++] = BYTE1(Acc.y);
//    data_to_send[_cnt++] = BYTE0(Acc.y);
    data_to_send[_cnt++] = BYTE1(Acc.z);
    data_to_send[_cnt++] = BYTE0(Acc.z);
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
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(USART1, data_to_send, _cnt);
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
    _temp = PID_ROL.P * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_ROL.I * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_ROL.D * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_PIT.P * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_PIT.I * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_PIT.D * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_YAW.P * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_YAW.I * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_YAW.D * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(USART1, data_to_send, _cnt);
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
    _temp = PID_ALT.P * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_ALT.I * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_ALT.D * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_POS.P * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_POS.I * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_POS.D * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_PID_1.P * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_PID_1.I * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_PID_1.D * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(USART1, data_to_send, _cnt);
#else
    NRF_TxPacket(data_to_send, _cnt);
#endif
}
void Data_Send_PID3(void)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x12;
    data_to_send[_cnt++] = 0;

    vs16 _temp;
    _temp = PID_PID_2.P * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_PID_2.I * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = PID_PID_2.D * 100;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

#ifdef DATA_TRANSFER_USE_USART
    Sys_sPrintf(USART1, data_to_send, _cnt);
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
    Sys_sPrintf(USART1, data_to_send, _cnt);
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
    Sys_sPrintf(USART1, data_to_send, _cnt);
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
    Sys_sPrintf(USART1, data_to_send, _cnt);
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

    if (Send_Status)
    {
        Send_Status = 0;
        Data_Send_Status();
    }
//    else if (Send_GpsData)
//    {
//        Send_GpsData = 0;
//        //Data_Send_GpsData();
//    }
    if (Send_Senser)
    {
        Send_Senser = 0;
        Data_Send_Senser();
    }
//    if (Send_PID1)
//    {
//        Send_PID1 = 0;
//        Data_Send_PID1();
//    }
//    else if (Send_PID2)
//    {
//        Send_PID2 = 0;
//        Data_Send_PID2();
//    }
//    else if (Send_PID3)
//    {
//        Send_PID3 = 0;
//        Data_Send_PID3();
//    }
//    if (Send_RC)
//    {
//        Send_RC = 0;
//        Data_Send_RC();
//    }
    if (Send_RCData)
    {
        Send_RCData = 0;
        Data_Send_RCData();
    }

//    if (Send_Offset)
//    {
//        Send_Offset = 0;
//        //Data_Send_OFFSET();
//    }
//    Send_MotoPwm = 1;
//    if (Send_MotoPwm)
//    {
//        Send_MotoPwm = 0;
//        Data_Send_MotoPWM();
//    }
}
