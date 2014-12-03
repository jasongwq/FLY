#ifndef _PFILTER_H_
#define _PFILTER_H_
#include "sys.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct
{
    u8 *pbuf;
    u8 *data;
    u16 num;
    u16 cnt;
    u32 sum;
} SLIDE_FILTERING8;
typedef struct
{
    s16 *pbuf;
    s16 data;
    u16 num;
    u16 cnt;
    s32 sum;
} SLIDE_FILTERING16;
typedef struct
{
    s32 *pbuf;
    s32 *data;
    u16 num;
    u16 cnt;
    s32 sum;
} SLIDE_FILTERING32;
typedef struct
{
    float *pbuf;
    float *data;
    u16 num;
    u16 cnt;
    double sum;
} SLIDE_FILTERINGF;
extern u8    slide_filtering8( SLIDE_FILTERING8  *dat);
extern u16   slide_filtering16(SLIDE_FILTERING16 *dat);
extern s32   slide_filtering32(SLIDE_FILTERING32 *dat);
extern float slide_filteringf( SLIDE_FILTERINGF  *dat);

#ifdef __cplusplus
}
#endif
#endif

