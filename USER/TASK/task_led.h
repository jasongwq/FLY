#define LED1_Init     PA2_OUT
#define LED1_Toggle {static char i = 0;PAout(2) = i++&0X01;}
#define LED1_H PAout(2) = 1
#define LED1_L PAout(2) = 0

int task_led(void)
{
    _SS
    LED1_Init;
    LED1_H;
    while (1)
    {
        WaitX(500);
        if (RC_Control.ARMED)
            LED1_H;
        else
            LED1_Toggle;
    }
    _EE
}

int loop_led(void)
{
    _SS
    LED1_Init;
    LED1_H;
    _LOOP_SS
    if (RC_Control.ARMED)
        LED1_H;
    else
        LED1_Toggle;
    LoopX(500);
    _EE
}
