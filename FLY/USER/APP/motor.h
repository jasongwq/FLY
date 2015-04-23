#ifndef __MOTOR_H_
#define __MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sys.h"

extern int32_t MOTO1_PWM, MOTO2_PWM, MOTO3_PWM, MOTO4_PWM;
extern void Moto_PwmRflash(int32_t MOTO1_PWM, int32_t MOTO2_PWM, int32_t MOTO3_PWM, int32_t MOTO4_PWM);

#ifdef __cplusplus
        }
#endif


#endif


