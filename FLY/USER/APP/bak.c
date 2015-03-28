#include "bak.h"
#include "stmflash.h"
#include "control.h"
#include "Balance.h"
void Data_Save(u8 select)
{
    RCC->APB1ENR |= 1 << 28;//RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC->APB1ENR |= 1 << 27;//RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);
    PWR->CR |= 1 << 8;      //DISENABLE写保护
    switch (select)
    {
    case 1:
        BKP_WriteBackupRegister(BKP_DR1,  (u16)(PID_ROL.P   * 100 ));
        BKP_WriteBackupRegister(BKP_DR2,  (u16)(PID_ROL.I   * 1000));
        BKP_WriteBackupRegister(BKP_DR3,  (u16)(PID_ROL.D   * 100 ));
        BKP_WriteBackupRegister(BKP_DR4,  (u16)(PID_PIT.P   * 100 ));
        BKP_WriteBackupRegister(BKP_DR5,  (u16)(PID_PIT.I   * 1000));
        BKP_WriteBackupRegister(BKP_DR6,  (u16)(PID_PIT.D   * 100 ));
        BKP_WriteBackupRegister(BKP_DR7,  (u16)(PID_YAW.P   * 100 ));
        BKP_WriteBackupRegister(BKP_DR8,  (u16)(PID_YAW.I   * 100 ));
        BKP_WriteBackupRegister(BKP_DR9,  (u16)(PID_YAW.D   * 100 ));
				
        BKP_WriteBackupRegister(BKP_DR10, (u16)(PID_ALT.P   * 1000));
        BKP_WriteBackupRegister(BKP_DR11, (u16)(PID_ALT.I   * 100 ));
        BKP_WriteBackupRegister(BKP_DR12, (u16)(PID_ALT.D   * 100 ));
				
        BKP_WriteBackupRegister(BKP_DR13, (u16)(PID_PID_1.P * 100 ));
        BKP_WriteBackupRegister(BKP_DR14, (u16)(PID_PID_1.I * 100));
        BKP_WriteBackupRegister(BKP_DR15, (u16)(PID_PID_1.D * 100 ));
        break;
    case 2:
        BKP_WriteBackupRegister(BKP_DR16, (u16)(ACC_OFFSET.x      ));
        BKP_WriteBackupRegister(BKP_DR17, (u16)(ACC_OFFSET.y      ));
        BKP_WriteBackupRegister(BKP_DR18, (u16)(ACC_OFFSET.z      ));
        break;
    case 3:
        BKP_WriteBackupRegister(BKP_DR19, (u16)(GYRO_OFFSET.x     ));
        BKP_WriteBackupRegister(BKP_DR20, (u16)(GYRO_OFFSET.y     ));
        BKP_WriteBackupRegister(BKP_DR21, (u16)(GYRO_OFFSET.z     ));
        break;
    }
    BKP_WriteBackupRegister(BKP_DR42, 1);
}
u16 Data_Read(void)
{
    PID_ROL.P  =  ((float)(BKP_ReadBackupRegister(BKP_DR1 )) / 100  );
    PID_ROL.I  =  ((float)(BKP_ReadBackupRegister(BKP_DR2 )) / 1000 );
    PID_ROL.D  =  ((float)(BKP_ReadBackupRegister(BKP_DR3 )) / 100  );
    PID_PIT.P  =  ((float)(BKP_ReadBackupRegister(BKP_DR4 )) / 100  );
    PID_PIT.I  =  ((float)(BKP_ReadBackupRegister(BKP_DR5 )) / 1000 );
    PID_PIT.D  =  ((float)(BKP_ReadBackupRegister(BKP_DR6 )) / 100  );
    PID_YAW.P  =  ((float)(BKP_ReadBackupRegister(BKP_DR7 )) / 100  );
    PID_YAW.I  =  ((float)(BKP_ReadBackupRegister(BKP_DR8 )) / 100  );
    PID_YAW.D  =  ((float)(BKP_ReadBackupRegister(BKP_DR9 )) / 100  );
		
    PID_ALT.P  =  ((float)(BKP_ReadBackupRegister(BKP_DR10)) / 1000 );
    PID_ALT.I  =  ((float)(BKP_ReadBackupRegister(BKP_DR11)) / 100  );
    PID_ALT.D  =  ((float)(BKP_ReadBackupRegister(BKP_DR12)) / 100  );
		
    PID_PID_1.P = ((float)(BKP_ReadBackupRegister(BKP_DR13)) / 100  );
    PID_PID_1.I = ((float)(BKP_ReadBackupRegister(BKP_DR14)) / 100 );
    PID_PID_1.D = ((float)(BKP_ReadBackupRegister(BKP_DR15)) / 100  );

    ACC_OFFSET.x  =  ((u8)(BKP_ReadBackupRegister(BKP_DR16))        );
    ACC_OFFSET.y  =  ((u8)(BKP_ReadBackupRegister(BKP_DR17))        );
    ACC_OFFSET.z  =  ((u8)(BKP_ReadBackupRegister(BKP_DR18))        );
    GYRO_OFFSET.x =  ((u8)(BKP_ReadBackupRegister(BKP_DR19))        );
    GYRO_OFFSET.y =  ((u8)(BKP_ReadBackupRegister(BKP_DR20))        );
    GYRO_OFFSET.z =  ((u8)(BKP_ReadBackupRegister(BKP_DR21))        );
    return BKP_ReadBackupRegister(BKP_DR42);
};

#define SIZE 21
#define FLASH_SAVE_ADDR  0X08035000      //设置FLASH 保存地址(必须为偶数)

void StmFlash_Save(void)
{
    u16 StmFlash_Buffer[SIZE];
    int i = 0;
    StmFlash_Buffer[i++] = (u16)(PID_ROL.P   * 100 );
    StmFlash_Buffer[i++] = (u16)(PID_ROL.I   * 1000);
    StmFlash_Buffer[i++] = (u16)(PID_ROL.D   * 100 );
    StmFlash_Buffer[i++] = (u16)(PID_PIT.P   * 100 );
    StmFlash_Buffer[i++] = (u16)(PID_PIT.I   * 1000);
    StmFlash_Buffer[i++] = (u16)(PID_PIT.D   * 100 );
    StmFlash_Buffer[i++] = (u16)(PID_YAW.P   * 100 );
    StmFlash_Buffer[i++] = (u16)(PID_YAW.I   * 1000);
    StmFlash_Buffer[i++] = (u16)(PID_YAW.D   * 100 );
    StmFlash_Buffer[i++] = (u16)(PID_ALT.P   * 1000);
    StmFlash_Buffer[i++] = (u16)(PID_ALT.I   * 100 );
    StmFlash_Buffer[i++] = (u16)(PID_ALT.D   * 100 );
    StmFlash_Buffer[i++] = (u16)(PID_PID_1.P * 100 );
    StmFlash_Buffer[i++] = (u16)(PID_PID_1.I * 100 );
    StmFlash_Buffer[i++] = (u16)(PID_PID_1.D * 100 );
    StmFlash_Buffer[i++] = (u16)(ACC_OFFSET.x      );
    StmFlash_Buffer[i++] = (u16)(ACC_OFFSET.y      );
    StmFlash_Buffer[i++] = (u16)(ACC_OFFSET.z      );
    StmFlash_Buffer[i++] = (u16)(GYRO_OFFSET.x     );
    StmFlash_Buffer[i++] = (u16)(GYRO_OFFSET.y     );
    StmFlash_Buffer[i++] = (u16)(GYRO_OFFSET.z     );

    STMFLASH_Write(FLASH_SAVE_ADDR, (u16 *)StmFlash_Buffer, SIZE);
}
void StmFlash_Read(void)
{
    u16 StmFlash_Buffer[SIZE];
    int i = 0;
    STMFLASH_Read(FLASH_SAVE_ADDR, (u16 *)StmFlash_Buffer, SIZE);
    PID_ROL.P  =  ((float)(StmFlash_Buffer[i++]) / 100  );
    PID_ROL.I  =  ((float)(StmFlash_Buffer[i++]) / 1000 );
    PID_ROL.D  =  ((float)(StmFlash_Buffer[i++]) / 100  );
    PID_PIT.P  =  ((float)(StmFlash_Buffer[i++]) / 100  );
    PID_PIT.I  =  ((float)(StmFlash_Buffer[i++]) / 1000 );
    PID_PIT.D  =  ((float)(StmFlash_Buffer[i++]) / 100  );
    PID_YAW.P  =  ((float)(StmFlash_Buffer[i++]) / 100  );
    PID_YAW.I  =  ((float)(StmFlash_Buffer[i++]) / 1000 );
    PID_YAW.D  =  ((float)(StmFlash_Buffer[i++]) / 100  );
    PID_ALT.P  =  ((float)(StmFlash_Buffer[i++]) / 1000 );
    PID_ALT.I  =  ((float)(StmFlash_Buffer[i++]) / 100  );
    PID_ALT.D  =  ((float)(StmFlash_Buffer[i++]) / 100  );
    PID_PID_1.P = ((float)(StmFlash_Buffer[i++]) / 100  );
    PID_PID_1.I = ((float)(StmFlash_Buffer[i++]) / 100  );
    PID_PID_1.D = ((float)(StmFlash_Buffer[i++]) / 100  );
    ACC_OFFSET.x  =  ((u8)(StmFlash_Buffer[i++])        );
    ACC_OFFSET.y  =  ((u8)(StmFlash_Buffer[i++])        );
    ACC_OFFSET.z  =  ((u8)(StmFlash_Buffer[i++])        );
    GYRO_OFFSET.x =  ((u8)(StmFlash_Buffer[i++])        );
    GYRO_OFFSET.y =  ((u8)(StmFlash_Buffer[i++])        );
    GYRO_OFFSET.z =  ((u8)(StmFlash_Buffer[i++])        );
}

