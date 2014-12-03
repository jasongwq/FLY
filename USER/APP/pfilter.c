#include "pfilter.h"

u8 slide_filtering8(SLIDE_FILTERING8 *dat)
{
    dat->sum = dat->sum - dat->pbuf[dat->cnt] + *dat->data;
    dat->pbuf[dat->cnt] = *dat->data;
    dat->cnt++; if (dat->cnt >= dat->num) dat->cnt = 0;
    return dat->sum / dat->num;
}
u16 slide_filtering16(SLIDE_FILTERING16 *dat)
{
    dat->sum = dat->sum - dat->pbuf[dat->cnt] + dat->data;
    dat->pbuf[dat->cnt] = dat->data;
    dat->cnt++; if (dat->cnt >= dat->num) dat->cnt = 0;
    return dat->sum / dat->num;
}
s32 slide_filtering32(SLIDE_FILTERING32 *dat)
{
    dat->sum = dat->sum - dat->pbuf[dat->cnt] + *dat->data;
    dat->pbuf[dat->cnt] = *dat->data;
    dat->cnt++; if (dat->cnt >= dat->num) dat->cnt = 0;
    return dat->sum / dat->num;
}
float slide_filteringf(SLIDE_FILTERINGF *dat)
{
    dat->sum = dat->sum - dat->pbuf[dat->cnt] + *dat->data;
    dat->pbuf[dat->cnt] = *dat->data;
    dat->cnt++; if (dat->cnt >= dat->num) dat->cnt = 0;
    return dat->sum / dat->num;
}



