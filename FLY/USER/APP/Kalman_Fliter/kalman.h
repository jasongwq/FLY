#ifndef _KALMAN_H
#define	_KALMAN_H
#ifdef __cplusplus
       extern "C" {
#endif

extern double KalmanFilter(const double ResrcData,
					double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction,int i);

#ifdef __cplusplus
        }
#endif

#endif


