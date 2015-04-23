/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "G32_timer_cap.h"
void (*TIM1_IRQ)(void);
void (*TIM2_IRQ)(void);
void (*TIM3_IRQ)(void);
void (*TIM4_IRQ)(void);
void (*TIM5_IRQ)(void);
void (*TIM6_IRQ)(void);
void (*TIM7_IRQ)(void);
void (*TIM1_BRK_IRQ)(void);
void (*TIM1_UP_IRQ)(void);
void (*TIM1_TRG_COM_IRQ)(void);
void (*TIM1_CC_IRQ)(void);
extern void SYS_UART_IQR(USART_TypeDef *USARTx);
void USART1_IRQHandler(void)  //串口中断函数
{
    // ATK_Usart1_IQR();
    SYS_UART_IQR(USART1);

}
void USART2_IRQHandler(void)  //串口中断函数
{
    // ATK_Usart2_IQR();
    SYS_UART_IQR(USART2);
}
void USART3_IRQHandler(void)  //串口中断函数
{
    //ATK_Usart3_IQR()
    SYS_UART_IQR(USART3);
}
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC1))
    {
        DMA_ClearITPendingBit(DMA1_IT_GL1);
    }
}
void TIM1_BRK_IRQHandler(void)
{
    TIM1_BRK_IRQ();
}

void TIM1_TRG_COM_IRQHandler(void)
{
    TIM1_TRG_COM_IRQ();
}
extern void TIM1_Cap_IRQ(void);

extern void TIM3_Cap_IRQ(void);
extern void TIM4_Cap_IRQ(void);
extern void TIM8_Cap_IRQ(void);


void TIM1_UP_IRQHandler(void)
{
    #include "G32_Timer_Cap_Config.h"
#if (CAP_TIM1_CH1)||(CAP_TIM1_CH2)||(CAP_TIM1_CH3)||(CAP_TIM1_CH4)
  extern void TIM1_Cap_IRQ(void);
	TIM1_Cap_IRQ();
#endif
}
void TIM1_CC_IRQHandler(void)
{
	#include "G32_Timer_Cap_Config.h"
#if (CAP_TIM1_CH1)||(CAP_TIM1_CH2)||(CAP_TIM1_CH3)||(CAP_TIM1_CH4)
  extern void TIM1_Cap_IRQ(void);
	TIM1_Cap_IRQ();
#endif
}
void TIM8_UP_IRQHandler(void)
{
    #include "G32_Timer_Cap_Config.h"
#if (CAP_TIM8_CH1)||(CAP_TIM8_CH2)||(CAP_TIM8_CH3)||(CAP_TIM8_CH4)
  extern void TIM8_Cap_IRQ(void);
	TIM8_Cap_IRQ();
#endif
}
void TIM8_CC_IRQHandler(void)
{
	#include "G32_Timer_Cap_Config.h"
#if (CAP_TIM8_CH1)||(CAP_TIM8_CH2)||(CAP_TIM8_CH3)||(CAP_TIM8_CH4)
  extern void TIM8_Cap_IRQ(void);
	TIM8_Cap_IRQ();
#endif
}
void TIM2_IRQHandler(void)
{
#include "G32_Timer_Cap_Config.h"
#if (CAP_TIM2_CH1)||(CAP_TIM2_CH2)||(CAP_TIM2_CH3)||(CAP_TIM2_CH4)
  extern void TIM2_Cap_IRQ(void);
	TIM2_Cap_IRQ();
#endif
}
void TIM3_IRQHandler(void)
{
    TIM3_Cap_IRQ();
}
void TIM4_IRQHandler(void)
{
    TIM4_Cap_IRQ();
}
//void TIM5_IRQHandler(void)
//{
//    if (TIM5->SR & TIM_IT_Update)
//    {
//        TIM5_IRQ();
//        TIM5->SR = ~TIM_FLAG_Update;
//    }
//}
//extern void TIM5_Cap_IRQ(void);
void TIM5_IRQHandler(void)
{
    //TIM5_Cap_IRQ();
    // TIM5_IRQ();
}
void TIM6_IRQHandler(void)
{
    if (TIM6->SR & TIM_IT_Update)
    {
        TIM6_IRQ();
        TIM6->SR = ~TIM_FLAG_Update;
    }
}
void TIM7_IRQHandler(void)
{
    if (TIM7->SR & TIM_IT_Update)
    {
        TIM7_IRQ();
        TIM7->SR = ~TIM_FLAG_Update;
    }
}
/*******************************************************************************
* Function Name  : USB_HP_CAN_TX_IRQHandler
* Description    : This function handles USB High Priority or CAN TX interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_HP_CAN1_TX_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    //ATK_USB_LP_CAN1_RX0_IQR();

}
void USBWakeUp_IRQHandler(void)
{
}
#include "sys_os.h"

#ifdef OS_CRITICAL_METHOD   //如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
//systick中断服务函数,使用ucos时用到
#include "usr_usart.h"
// int mytime;
u32 SysTick_Time=0;
void SysTick_Handler(void)
{
		SysTick_Time++;if(SysTick_Time>=(OS_TICKS_PER_SEC*60*60))SysTick_Time=0;
    // static int i = 0;
    // if (i++ >= 1000)
    // {
    //     i = 0;
    //     //time
    //     Sys_Printf(USART1, "\r\n%d", mytime);
    //     mytime = 0;
    // }
    UpdateTimers();
    //RunTask(task5, 5); //任务0具有精确按时获得执行的权限，要求：task0每次执行消耗时间<0.5个 ticket
}
#endif

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
