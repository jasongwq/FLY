/***SYS***/
#include "user.h"
#include "sys.h"
#include "minos_delay.h"
#include "sys_os.h"
#include "gpio.h"
#include "stm32_config.h"
#include "G32_timer.h"
/***C***/
#include "string.h"
/***Other***/
void SYS_INIT(void)
{
    extern void NVIC_Configuration(void);
    /***延时初始化***/
    delay_init();
    /***中断初始化***/
    NVIC_Configuration();
    uart_init (115200);
    uart3_init(115200);

    Sys_Printf(USART1, "\r\nUSART1 ok");
    Sys_Printf(Printf_USART, "\r\nUSART1 ok");
    //Sys_Printf(Printf_USART, "\r\nUSART1 ok");

    delay_ms(500);
}

#include "usr_usart.h"
#include "kalman.h"
#include "MPU6050.h"
#include "IMU.h"
#include "stdio.h"
#include "filter.h"
#include "IMU.h"
#include "Control.h"
#include "Balance.h"
#include "rc.h"
#include "bak.h"
#include "./TASK/task_led.h"
int flag_ACC = 0;
#include "./TASK/task_6050.h"
#include "./TASK/task_ultrasonic.h"
#include "./TASK/task_pwm_ex.h"
#include "./TASK/task_cap_rc.h"
#include "./TASK/task_bmp085.h"
#include "./TASK/loop_fast.h"
#include "./TASK/task_hmc5883l.h"

int main(void)
{
    SYS_INIT();
    /***总循环***/
    while (1)
    {
        RunLoop(loop_fast, 0);
        //        RunLoop(loop_led,1);

        RunTaskA(task_6050, 2);

        RunTaskA(task_ultrasonic, 3);
        ////PWM
        ////A0
        ////A1
        ////C6
        ////C7
        RunTaskA(task_pwm_ex, 4);

        ////PB 0 1 4 5
        ////PD 12 13
#if VERSIONS ==1
        // RunTaskA(task_cap_rc, 5);
#endif
#if VERSIONS ==2
        RunTaskA(task_cap_rc, 5);
#endif
        //        RunTaskA(task_hmc5883l, 6);

        //        RunTaskA(task_bmp085, 7);
        //
        RunTaskA(task_led, 8);
    }
}
