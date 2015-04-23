#ifndef _FILTER_H_
#define _FILTER_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "IMU.h"
//typedef struct
//{
//    u8 *pbuf;
//    u8  data;
//    u16 num;
//    u16 cnt;
//    u32 sum;
//} SLIDE_FILTERING8;
//typedef struct
//{
//    u16 *pbuf;
//    u16 data;
//    u16 num;
//    u16 cnt;
//    u32 sum;
//} SLIDE_FILTERING16;
//typedef struct
//{
//    float *pbuf;
//    float data;
//    u16 num;
//    u16 cnt;
//    double sum;
//} SLIDE_FILTERINGF;
//extern u8 slide_filtering8(SLIDE_FILTERING8 dat);
//extern u16 slide_filtering16(SLIDE_FILTERING16 dat);
//extern float slide_filteringf(SLIDE_FILTERINGF dat);


extern void Prepare_Data(S_INT16_XYZ *acc_in, S_INT16_XYZ *acc_out);
extern void Prepare_Data2(T_float_angle *angle_in);
extern void Prepare_Data3(S_INT16_XYZ *acc_in, S_INT16_XYZ *gyro_in, S_INT16_XYZ *acc_out);
#ifdef __cplusplus
        }
#endif

#endif

