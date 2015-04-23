#ifdef __cplusplus
       extern "C" {
#endif
/*
    Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
    R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
*/
//float KALMAN_Q    0.001
//float KALMAN_R    10.0000
//float KALMAN_P0   2210.0000
int flag_ACC = 0;
int task_6050(void)
{
    extern S_INT16_XYZ Acc, Gyr, Mag, Average_Acc;
    extern S_INT16_XYZ Average2_Acc, Average3_Acc, Average1_Gyr, Average2_Gyr;

    _SS
    MPU6050_Init();
//    if (0 == Data_Read())
//    {
//        StmFlash_Read();
//    }
    WaitX(200);
    while (1)
    {
        WaitX(1);
        MPU6050_Read();
        MPU6050_Dataanl();//5us

        Acc.x = MPU6050_ACC_LAST.x;
        Acc.y = MPU6050_ACC_LAST.y;
        Acc.z = MPU6050_ACC_LAST.z;
        Gyr.x = MPU6050_GYRO_LAST.x;
        Gyr.y = MPU6050_GYRO_LAST.y;
        Gyr.z = MPU6050_GYRO_LAST.z;

//        Average_Acc.x=KalmanFilter((double)Acc.x, *KALMAN_Q, *KALMAN_R, *KALMAN_P0,0);
//        Average_Acc.y=KalmanFilter((double)Acc.y, *KALMAN_Q, *KALMAN_R, *KALMAN_P0,1);
//        Average_Acc.z=KalmanFilter((double)Acc.z, *KALMAN_Q, *KALMAN_R, *KALMAN_P0,2);

        Prepare_Data(&Acc, &Average_Acc);//4us
        flag_ACC = 1;
//				{
//                static u32 currenttime = 0;
//                static int i;
//								u32 lasttime = 0;
//                lasttime = currenttime;
//                currenttime = SysTick_Clock();
//                if(i++>10){i=0;Sys_Printf(Printf_USART, "\r\n %d", currenttime - lasttime);}
//            }
    }
    _EE
}
#ifdef __cplusplus
        }
#endif
