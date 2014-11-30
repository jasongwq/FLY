#include "sys.h"
#include "pfilter.h"

u8 slide_filtering(SLIDE_FILTERING8 dat)
{
    dat.sum = dat.sum - dat.pbuf[dat.cnt] + dat.data;
    dat.pbuf[dat.cnt] = dat.data;
    dat.cnt++; if (dat.cnt == dat.num) dat.cnt = 0;
    return dat.sum / dat.num;
}
u16 slide_filtering(SLIDE_FILTERING16 dat)
{
    dat.sum = dat.sum - dat.pbuf[dat.cnt] + dat.data;
    dat.pbuf[dat.cnt] = dat.data;
    dat.cnt++; if (dat.cnt == dat.num) dat.cnt = 0;
    return dat.sum / dat.num;
}
float slide_filtering(SLIDE_FILTERINGF dat)
{
    dat.sum = dat.sum - dat.pbuf[dat.cnt] + dat.data;
    dat.pbuf[dat.cnt] = dat.data;
    dat.cnt++; if (dat.cnt == dat.num) dat.cnt = 0;
    return dat.sum / dat.num;
}



