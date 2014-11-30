#ifndef _PFILTER_H_
#define _PFILTER_H_
#include "sys.h"
typedef struct
{
    u8 *pbuf;
    u8  data;
    u16 num;
    u16 cnt;
    u32 sum;
} SLIDE_FILTERING8;
typedef struct
{
    u16 *pbuf;
    u16 data;
    u16 num;
    u16 cnt;
    u32 sum;
} SLIDE_FILTERING16;
typedef struct
{
    float *pbuf;
    float data;
    u16 num;
    u16 cnt;
    double sum;
} SLIDE_FILTERINGF;
extern u8    slide_filtering(SLIDE_FILTERING8 dat);
extern u16   slide_filtering(SLIDE_FILTERING16 dat);
extern float slide_filtering(SLIDE_FILTERINGF dat);


#endif

