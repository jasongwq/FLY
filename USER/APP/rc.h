#ifndef _RC_H_
#define _RC_H_
#ifdef __cplusplus
       extern "C" {
#endif

#include "sys.h"
#include "data_transfer.h"

typedef struct
{
    u8 ARMED;
    u8 Constant_Level;
    u8 ALT_ON_OFF;
} T_Control;

extern T_RC_Data Rc_Data;//1000~2000
extern T_RC_Data Rc_D;
extern T_Control RC_Control;

extern void Rc_DataAnl(u16 Rc_Pwm_In[5]);
void RC_Analyse(T_RC_Data *rc_data, T_Control *ctl_data);
void Rc_GetValue(T_RC_Data *temp);

#define RC_FUN_MIN  1200
#define RC_FUN_MAX  1800
#define RC_FUN_CNT  20
#define RC_FUN_ZERO 1000
#ifdef __cplusplus
        }
#endif
#endif


