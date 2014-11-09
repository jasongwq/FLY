#ifndef _KALMAN_H
#define _KALMAN_H
#include "sys.h"

#define   LENGTH      1*1
#define   ORDER       1
//#define   N           100
#define   SEED        1567


extern void   Random(unsigned long Num, float *S, float mu, float sigma);
extern void   KalMan(u16* in,u16* out);


#endif

