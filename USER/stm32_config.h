#ifndef __STM32_CONFIG_
#define __STM32_CONFIG_
#ifdef __cplusplus
       extern "C" {
#endif
#define VERSIONS 2
/* 
1 黑板
2 yyb绿板
*/
#define ROTATE90 0
/*
1 
Rothmarketing 90 CCW
方向四轴旋转90度
0
不旋转
*/
#define THROTTLE_INVERTED 1 //throttleinverted 1
/*
1 反转
0 不反转
油门反相 学长遥控需要反相
*/

/*
#include "stm32_config.h"
*/
/***I2C CONGIG ***/
#include "G32_I2c_Config.h"
/***I2C CONGIG ***/
/****************************************/
/***24L01配置***/
#include "G32_Nrf24l01_Config.h"
/***24L01配置***/
/****************************************/
/***Ultrasonic***/
#include "Ultrasonic_Config.h"
/***Ultrasonic***/
/****************************************/
/***USART***/
#include "Usart_Config.h"
/***USART***/
/****************************************/
/*** LCD ***/
/******************************/
/***ourstm400240 CONGIG ***/
#include "ourstm400240.h"
/***ourstm400240 CONGIG ***/
/******************************/
/*** LCD ***/
/****************************************/
/***TOUCH***/
/***TOUCH***/
/****************************************/
/***ADC***/
#include "Adc_Config.h"
/***ADC***/
/****************************************/
/***PWM***/
#include "Pwm_Config.h"
/***PWM***/
/****************************************/
/***MPU6050***/
//#define MPU6050_USE_DMP
/***MPU6050***/
/****************************************/
/***SD***/
/***SD***/
/****************************************/
/***FATFS***/
/***FATFS***/
/****************************************/
/***DS18B20***/
#include "DS18B20_Config.h"
/***DS18B20***/
/****************************************/
#ifdef __cplusplus
        }
#endif
#endif

