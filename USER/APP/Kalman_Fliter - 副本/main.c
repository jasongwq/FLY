#include  "systemInit.h"


#define   UART_PERIPH       SYSCTL_PERIPH_UART0
#define   UART_USE_BASE     UART0_BASE
#define   GPIO_PERIPH       SYSCTL_PERIPH_GPIOA
#define   GPIO_USE_BASE     GPIO_PORTA_BASE
#define   GPIO_PINS         GPIO_PIN_0|GPIO_PIN_1
#define   BAUDRATE          9600
#define   MAXNUM            100


unsigned char ADC_EndFlag = 0;
//============================================================================//
//==                        ADC中断函数                                     ==//
//============================================================================//
//==入口参数: 无                                                            ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void ADC_ISR(void)
{
  unsigned long ulStatus;
  ulStatus = ADCIntStatus(ADC_BASE, 0, true);
  ADCIntClear(ADC_BASE, 0);
  
  if (ulStatus)
  {
    ADC_EndFlag = true;
  }
}





//============================================================================//
//==                          UART初始化函数                                ==//
//============================================================================//
//==入口参数: 无                                                            ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void UARTInit(void)
{
  SysCtlPeripheralEnable(GPIO_PERIPH);        //使能UART所在GPIO端口
  SysCtlPeripheralEnable(UART_PERIPH);        //使能UART外设
  GPIOPinTypeUART(GPIO_USE_BASE, GPIO_PINS);  //使能UART所在引脚
  
  UARTConfigSet(UART_USE_BASE, BAUDRATE, UART_CONFIG_WLEN_8|
                                         UART_CONFIG_STOP_ONE|
                                         UART_CONFIG_PAR_NONE);
  UARTEnable(UART_USE_BASE);                  //使能UART
}





//============================================================================//
//==                      ADC初始化函数                                     ==//
//============================================================================//
//==说明:     对于测试将方波滤波成为正弦波的测试,需要改变AD的采样频率       ==//
//==          SYSCTL_ADCSPEED_1MSPS   // 采样速率：1M次采样/秒              ==//
//==          SYSCTL_ADCSPEED_500KSPS // 采样速率：500K次采样/秒            ==//
//==          SYSCTL_ADCSPEED_250KSPS // 采样速率：250K次采样/秒            ==//
//==          SYSCTL_ADCSPEED_125KSPS // 采样速率：125K次采样/秒            ==//
//==          频率高的方波AD采样频率应适当增加                              ==//
//==入口参数: 无                                                            ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void ADCInit(void)
{
  SysCtlPeriEnable(SYSCTL_PERIPH_ADC);                 // 使能ADC模块
  SysCtlADCSpeedSet(SYSCTL_ADCSPEED_500KSPS);          // 设置ADC采样速率
  ADCSequDisable(ADC_BASE, 0);                         // 配置前先禁止采样序列
// 采样序列配置：ADC基址，采样序列编号，触发事件，采样优先级
  ADCSequConfig(ADC_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
// 采样步进设置：ADC基址，采样序列编号，步值，通道设置(ADC0转换完后停止触发中断)
  ADCSequStepConfig(ADC_BASE, 0, 0, ADC_CTL_CH0 |
                                    ADC_CTL_END |
                                    ADC_CTL_IE);
  ADCIntEnable(ADC_BASE, 0);                           // 使能ADC中断
  IntEnable(INT_ADC0);                                 // 使能ADC采样序列中断
  ADCIntRegister(ADC_BASE, 0, ADC_ISR);
  IntMasterEnable( );                                  // 使能处理器中断
  ADCSequEnable(ADC_BASE, 0);                          // 使能采样序列
}





//============================================================================//
//==                        ADC采样函数                                     ==//
//============================================================================//
//==函数说明: 以中断方式采样AD                                              ==//
//==入口参数: 无                                                            ==//
//==出口参数: 无                                                            ==//
//==返回值:   本次采样值(单位为电压mV)                                      ==//
//============================================================================//
unsigned short ADCSampling(void)
{
  unsigned long   ulValue;
  unsigned short  usValue;
  
  ADCProcessorTrigger(ADC_BASE, 0);
  while(!ADC_EndFlag);
  ADC_EndFlag = false;
  ADCSequDataGet(ADC_BASE, 0, &ulValue);
  ulValue = (ulValue*3000)/1024;
  usValue = ulValue;
  
  return usValue;
}





//============================================================================//
//==                          长整型转换函数                                ==//
//============================================================================//
//==入口参数: LongNum		指定的长整型                                ==//
//==	      *pArray		指向保存转换后的数据(四字节)                ==//
//==出口参数: *pArray		指向保存转换后的数据(四字节)                ==//
//==返回值:   无                                                            ==//
//============================================================================//
void Long2Char(unsigned long longNum, unsigned char *pArray)
{
  unsigned char *pChar=NULL;
  unsigned char i=0;
  
  pChar = (unsigned char*)(&longNum);
  for (i=0; i<sizeof(long); i++)
  {
    *pArray++ = *pChar++;
  }
}





//============================================================================//
//==                          长整型数发送函数                              ==//
//============================================================================//
//==入口参数: *LArray         指向需要发送的长整型数数组                    ==//
//==          Num             长整型数据个数                                ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void LongSend(unsigned long *LArray, unsigned int Num)
{
  unsigned int i;
  unsigned char a[4] = {0};
  
  
  for (i=0; i<Num; i++)
  {
    Long2Char(LArray[i], a);
    UARTCharPut(UART_USE_BASE, a[0]);   //低字节放在低地址,先发送低字节
    UARTCharPut(UART_USE_BASE, a[1]);
    UARTCharPut(UART_USE_BASE, a[2]);
    UARTCharPut(UART_USE_BASE, a[3]);
  }
}





//============================================================================//
//==                            整型转换函数                                ==//
//============================================================================//
//==入口参数: ShortNum		指定的整型                                  ==//
//==	      *pArray		指向保存转换后的数据(四字节)                ==//
//==出口参数: *pArray		指向保存转换后的数据(四字节)                ==//
//==返回值:   无                                                            ==//
//============================================================================//
void Short2Char(unsigned short ShortNum, unsigned char *pArray)
{
  unsigned char *pChar=NULL;
  unsigned char i=0;
  
  pChar = (unsigned char*)(&ShortNum);
  for (i=0; i<sizeof(short); i++)
  {
    *pArray++ = *pChar++;
  }
}





//============================================================================//
//==                            整型数发送函数                              ==//
//============================================================================//
//==入口参数: *SArray         指向需要发送的整型数数组                      ==//
//==          Num             整型数据个数                                  ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void ShortSend(unsigned short *SArray, unsigned int Num)
{
  unsigned int i;
  unsigned char a[2] = {0};
  
  
  for (i=0; i<Num; i++)
  {
    Short2Char(SArray[i], a);
    UARTCharPut(UART_USE_BASE, a[0]);   //低字节放在低地址,先发送低字节
    UARTCharPut(UART_USE_BASE, a[1]);
  }
}





//============================================================================//
//==                           浮点数转换函数                               ==//
//============================================================================//
//==入口参数: FloatNum		指定的浮点数                                ==//
//==	      *pArray		指向保存转换后的数据(四字节)                ==//
//==出口参数: *pArray		指向保存转换后的数据(四字节)                ==//
//==返回值:   无                                                            ==//
//============================================================================//
void Float2Char(float FloatNum, unsigned char *pArray)
{
  unsigned char *pChar=NULL;
  unsigned char i=0;
  
  pChar = (unsigned char*)(&FloatNum);
  for (i=0; i<sizeof(float); i++)
  {
    *pArray++ = *pChar++;
  }
}





//============================================================================//
//==                           浮点数发送函数                               ==//
//============================================================================//
//==入口参数: *Farray		指向需要发送的浮点数数组                    ==//
//==	      Num		浮点数个数				    ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void FloatSend(float *FArray, unsigned int Num)
{
  unsigned char i;
  unsigned char a[4] = {0};
  
  
  for (i=0; i<Num; i++)
  {
    Float2Char(FArray[i], a);
    UARTCharPut(UART_USE_BASE, a[0]);
    UARTCharPut(UART_USE_BASE, a[1]);
    UARTCharPut(UART_USE_BASE, a[2]);
    UARTCharPut(UART_USE_BASE, a[3]);
  }
}

  


//  主函数（程序入口）
int main(void)
{
    jtagWait();                                             //  防止JTAG失效，重要！
    clockInit();                                            //  时钟初始化：晶振，6MHz
    UARTInit();
    ADCInit();
    
    KalMan();
    for (;;)
    {
    }
}
