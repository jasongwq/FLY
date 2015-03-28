#include "filter.h"

#include "MPU6050.h"
#include <math.h>
#define Gyr_Gain 0.061     // 满量程2000dps时灵敏度(dps/digital)
#define pi 3.14159
#include "control.h"


#define FILTER_NUM  10
void Prepare_Data(S_INT16_XYZ *acc_in, S_INT16_XYZ *acc_out)
{
    static uint8_t  filter_cnt = 0;
    static int16_t  ACC_X_BUF[FILTER_NUM], ACC_Y_BUF[FILTER_NUM], ACC_Z_BUF[FILTER_NUM];
    static int32_t  temp1 = 0, temp2 = 0, temp3 = 0;
    uint8_t i;

    if (ACC_Z_BUF[FILTER_NUM - 1] == 0)
    {
        temp1 = temp2 = temp3 = 0;
        ACC_X_BUF[filter_cnt] = acc_in->x;
        ACC_Y_BUF[filter_cnt] = acc_in->y;
        ACC_Z_BUF[filter_cnt] = acc_in->z;
        for (i = 0; i < FILTER_NUM; i++)
        {
            temp1 += ACC_X_BUF[i];
            temp2 += ACC_Y_BUF[i];
            temp3 += ACC_Z_BUF[i];
        }
    }
    else
    {
        temp1 = temp1 - ACC_X_BUF[filter_cnt] + acc_in->x;
        temp2 = temp2 - ACC_Y_BUF[filter_cnt] + acc_in->y;
        temp3 = temp3 - ACC_Z_BUF[filter_cnt] + acc_in->z;

        ACC_X_BUF[filter_cnt] = acc_in->x;
        ACC_Y_BUF[filter_cnt] = acc_in->y;
        ACC_Z_BUF[filter_cnt] = acc_in->z;
    }
    acc_out->x = temp1 / FILTER_NUM;
    acc_out->y = temp2 / FILTER_NUM;
    acc_out->z = temp3 / FILTER_NUM;
    filter_cnt++;
    if (filter_cnt == FILTER_NUM)  filter_cnt = 0;
}
#define FILTER_NUM2     2
#define FILTER_NUM3     10
void Prepare_Data2(T_float_angle *angle_in)
{
    static uint8_t  filter_cnt = 0;
    static uint8_t  filter_cnt2 = 0;
    static float    ACC_X_BUF[FILTER_NUM2], ACC_Y_BUF[FILTER_NUM2], ACC_Z_BUF[FILTER_NUM3];
    float temp1 = 0, temp2 = 0, temp3 = 0;
    uint8_t i;
    ACC_X_BUF[filter_cnt]  = angle_in->pit;
    ACC_Y_BUF[filter_cnt]  = angle_in->rol;
    ACC_Z_BUF[filter_cnt2] = angle_in->yaw;
    for (i = 0; i < FILTER_NUM2; i++)
    {
        temp1 += ACC_X_BUF[i];
        temp2 += ACC_Y_BUF[i];
    }
    for (i = 0; i < FILTER_NUM3; i++)
    {
        temp3 += ACC_Z_BUF[i];
    }
    angle_in->pit = temp1 / FILTER_NUM2;
    angle_in->rol = temp2 / FILTER_NUM2;
    angle_in->yaw = temp3 / FILTER_NUM3;
    filter_cnt++;
    filter_cnt2++;
    if (filter_cnt == FILTER_NUM2)  filter_cnt = 0;
    if (filter_cnt2 == FILTER_NUM3) filter_cnt2 = 0;
}



//void Prepare_Data3(S_INT16_XYZ *acc_in, S_INT16_XYZ *gyro_in, S_INT16_XYZ *acc_out)
//{
//   //    static float Com_angley;
//   //    static float Com_anglex;
//   //        static float angleG;
//   float omegax =  Gyr_Gain * (gyro_in->X);
//   float omegay =  Gyr_Gain * (-gyro_in->Y);
//   //        float omegaz =  Gyr_Gain * (gyro_in->Z);
//   //
//   float dt = 1 / 1000.0;  // 微分时间(s)
//   float angleAy = atan2(acc_in->Y, acc_in->Z) * 180 / pi;
//   float angleAx = atan2(acc_in->X, acc_in->Z) * 180 / pi;

//   // 一阶互补算法
//   float K;
//   K = PID_PID_1.I;                              // 对加速度计取值的权重
//   /*    float A = K / (K + dt);

//       Com_anglex = A * (Com_anglex + omegay * dt) + (1 - A) * angleAx;
//       //acc_in->X = (int16_t)(Com_anglex);
//       //    //        acc_out->Y=(int16_t)(angleAx);
//       //    //        angleG = angleG + omegay * dt;           // 对角速度积分得到的角度(degree)
//       //    //        acc_out->Z=(int16_t)(angleG);
//       Com_angley = A * (Com_angley + omegax * dt) + (1 - A) * angleAy;
//       //acc_in->Y = (int16_t)(Com_angley);

//       acc_out->X = sin(Com_anglex * pi / 180) * 7568.0;
//       acc_out->Y = Com_angley;
//       acc_out->Z = Com_anglex;

//   */
//          acc_out->Z = acc_in->X;

//       static float y1x;
//       static float Com2_anglex;
//       //// 二阶互补算法
//   //    K = 0;
//       float x1 = (angleAx - Com2_anglex) * K * K;
//       y1x = y1x + x1 * dt;
//       float x2 = y1x + 2 * K * (angleAx - Com2_anglex) + omegax;
//       Com2_anglex = Com2_anglex + x2 * dt;
//       acc_out->X=Com2_anglex*100;
//   // 卡尔曼滤波
////    Klm_angle += (omega - bias) * dt;          // 先验估计
////    P_00 += -(P_10 + P_01) * dt + Q_angle * dt;
////    P_01 += -P_11 * dt;
////    P_10 += -P_11 * dt;
////    P_11 += +Q_omega * dt;                     // 先验估计误差协方差

////    float K_0 = P_00 / (P_00 + R_angle);
////    float K_1 = P_10 / (P_00 + R_angle);

////    bias += K_1 * (angleA - Klm_angle);
////    Klm_angle += K_0 * (angleA - Klm_angle);   // 后验估计
////    P_00 -= K_0 * P_00;
////    P_01 -= K_0 * P_01;
////    P_10 -= K_1 * P_00;
////    P_11 -= K_1 * P_01;                        // 后验估计误差协方差
//}


//u8 slide_filtering8(SLIDE_FILTERING8 dat)
//{
//    dat.sum = dat.sum - dat.pbuf[dat.cnt] + dat.data;
//    dat.pbuf[dat.cnt] = dat.data;
//    dat.cnt++; if (dat.cnt == dat.num) dat.cnt = 0;
//    return dat.sum / dat.num;
//}
//u16 slide_filtering16(SLIDE_FILTERING16 dat)
//{
//    dat.sum = dat.sum - dat.pbuf[dat.cnt] + dat.data;
//    dat.pbuf[dat.cnt] = dat.data;
//    dat.cnt++; if (dat.cnt == dat.num) dat.cnt = 0;
//    return dat.sum / dat.num;
//}
//float slide_filteringf(SLIDE_FILTERINGF dat)
//{
//    dat.sum = dat.sum - dat.pbuf[dat.cnt] + dat.data;
//    dat.pbuf[dat.cnt] = dat.data;
//    dat.cnt++; if (dat.cnt == dat.num) dat.cnt = 0;
//    return dat.sum / dat.num;
//}



