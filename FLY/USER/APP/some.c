#include "some.h"
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

u8 RxState = 0;
u8 RxBuffer[256];
u8 Step = 0;
u8 UP_EN = 0, DOWN_EN = 0, LEFT_EN = 0, RIGHT_EN = 0 ;
#include "usr_usart.h"
void some(USART_TypeDef *USARTx)
{
    //接收中断 (接收寄存器非空)
    if (USARTx->SR & (1 << 5)) //if(USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET)
    {
        u8 com_data = USARTx->DR;
        static u8 _data_len = 0, _data_cnt = 0;
        //Sys_Printf(USART2,"1");
        if (RxState == 0 && com_data == 0xAA)
        {
            //Sys_Printf(USART2,"aa");
            RxState = 1;
            RxBuffer[_data_len++] = com_data;
        }
        else if (RxState == 1 && com_data == 0xAA)
        {
            //Sys_Printf(USART2,"a2");
            RxState = 2;
            RxBuffer[_data_len++] = com_data;
        }
        else if (RxState == 2 && com_data > 0 && com_data < 0XF1)
        {
            //Sys_Printf(USART2,"b");
            RxState = 3;
            RxBuffer[_data_len++] = com_data;
        }
        else if (RxState == 3 && com_data < 50)
        {
            //Sys_Printf(USART2,"c");
            RxState = 4;
            RxBuffer[_data_len++] = com_data;
            _data_cnt = com_data;
        }
        else if (RxState == 4 && _data_len > 0)
        {
            _data_cnt--;
            RxBuffer[_data_len++] = com_data;
            if (_data_cnt == 0)
            {
                //SYS_USART_SendBuf(USART2,RxBuffer,_data_len);
                //Sys_Printf(USART2,"2");
                Data_Receive_Anl(RxBuffer, _data_len);
                RxState = 0;
                _data_len = 0;
            }
        }
        else
        {
            RxState = 0;
            _data_len = 0;
        }
    }
}
void TEST(void)
{
    Sys_Printf(USART2, "\r\n%d,%d,%d,%d,%d", UP_EN, DOWN_EN, LEFT_EN, RIGHT_EN, Step);
    UP_EN = DOWN_EN = LEFT_EN = RIGHT_EN = Step = 0;
}
void Data_Receive_Anl(u8 *data_buf, u8 num)
{
    if (*(data_buf + 2) == 0X01)
    {
        if (*(data_buf + 5) == 0X01)
        {
            UP_EN = 1;
        }
        if (*(data_buf + 5) == 0x02)
        {
            DOWN_EN = 1;
        }
        if (*(data_buf + 5) == 0x03)
        {
            LEFT_EN = 1;
        }
        if (*(data_buf + 5) == 0X04)
        {
            RIGHT_EN = 1;
        }
        Step = *(data_buf + 6);
        //Sys_Printf(USART2, "\r\n%d",*(data_buf + 5));
        TEST();
    }
}

//void SendValue()
//{
//      u8 data_to_send[32];
//    u8 _cnt = 0, sum = 0, i;
//    data_to_send[_cnt++] = 0xAA;
//    data_to_send[_cnt++] = 0xAA;
//    data_to_send[_cnt++] = 0x10;
//    data_to_send[_cnt++] = 0;
//    data_to_send[_cnt++] = BYTE1(up);
//    data_to_send[_cnt++] = BYTE0(up);
//    data_to_send[_cnt++] = BYTE1(down);
//    data_to_send[_cnt++] = BYTE0(down);
//    data_to_send[_cnt++] = BYTE1(left);
//    data_to_send[_cnt++] = BYTE0(left);
//    data_to_send[_cnt++] = BYTE1(right);
//    data_to_send[_cnt++] = BYTE0(right);
//    for (i = 0; i < _cnt; i++)
//        sum += data_to_send[i];
//    data_to_send[_cnt++] = sum;
//    data_to_send[3] = _cnt;
//    SYS_USART_SendBuf(DEBUG_USART,data_to_send, _cnt);
//}
