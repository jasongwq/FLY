#ifndef  __SYSTEM_INIT_H__
#define  __SYSTEM_INIT_H__


//  包含必要的头文件
//#include  <hw_types.h>
//#include  <hw_memmap.h>
//#include  <hw_ints.h>
//#include  <interrupt.h>
//#include  <sysctl.h>
//#include  <gpio.h>
//#include  <uart.h>
//#include  <adc.h>
//#include  "main.h"


#include  "kalman.h"
#include  "matrix.h"
//#include  "stdlib.h"
#include  "math.h"

//  将较长的标识符定义成较短的形式
#define  SysCtlPeriEnable       SysCtlPeripheralEnable
#define  SysCtlPeriDisable      SysCtlPeripheralDisable
#define  GPIOPinTypeIn          GPIOPinTypeGPIOInput
#define  GPIOPinTypeOut         GPIOPinTypeGPIOOutput
#define  GPIOPinTypeOD          GPIOPinTypeGPIOOutputOD
#define  ADCSequEnable          ADCSequenceEnable
#define  ADCSequDisable         ADCSequenceDisable
#define  ADCSequConfig          ADCSequenceConfigure
#define  ADCSequStepConfig      ADCSequenceStepConfigure
#define  ADCSequDataGet         ADCSequenceDataGet


//  声明全局的系统时钟变量
extern unsigned long TheSysClock;


//  防止JTAG失效
extern void jtagWait(void);


//  系统时钟初始化
extern void clockInit(void);


#endif  //  __SYSTEM_INIT_H__

