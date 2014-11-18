#ifndef __BAK_H_
#define __BAK_H_
#ifdef __cplusplus
       extern "C" {
#endif
#include "sys.h"

void Data_Save(u8 select);
u16 Data_Read(void);
void StmFlash_Read(void);
void StmFlash_Save(void);
#ifdef __cplusplus
        }
#endif
#endif


