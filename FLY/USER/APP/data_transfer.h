#ifndef _DATA_TRANSFER_H_
#define _DATA_TRANSFER_H_
#include "sys.h"
typedef struct int16_rcget
{
    int16_t ROLL;
    int16_t PITCH;
    int16_t THROTTLE;
    int16_t YAW;
    int16_t AUX1;
    int16_t AUX2;
    int16_t AUX3;
    int16_t AUX4;
    int16_t AUX5;
    int16_t AUX6;
} T_RC_Data;
struct DATA_TRANSFER_SWITCH
{
    char Status;
    char Senser;
    char RCData;
    char GpsData;
    char Offset;
    //char Offset;
    char PID1;
    char PID2;
    char PID3;
    char PID4;
    char PID5;
    char PID6;
    char MotoPwm;
    char DataF1;
    char DataF2;
    char DataF3;
    char DataF4;
};
extern struct DATA_TRANSFER_SWITCH Ex_ON_OFF, Send;
extern u8 Data_Check;
extern void Data_Exchange(void);
extern void Data_Send_Senser(void);
extern void Ex_Init (void);

extern float *yaw_just;
extern float *rol_just;
extern float *pit_just;

extern float *KALMAN_Q ;
extern float *KALMAN_R ;
extern float *KALMAN_P0 ;

#endif

