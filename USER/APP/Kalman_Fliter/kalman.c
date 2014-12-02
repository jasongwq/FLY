
#include "kalman.h"

/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
*/

double KalmanFilter(const double ResrcData,
					double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction,int i)
{
	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	static	double x_last[3];
  static	double p_last[3];

	double x_mid = x_last[i];
	double x_now;

	

	double p_mid ;
	double p_now;
	double kg;

	x_mid=x_last[i]; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid=p_last[i]+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
	x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
		
	p_now=(1-kg)*p_mid;//最优值对应的covariance	

	p_last[i] = p_now; //更新covariance值
	x_last[i] = x_now; //更新系统状态值

	return x_now;		
}





