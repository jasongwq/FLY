#include  "systemInit.h"


/*==============================================================================
1.预估计
   X(k|k-1) = F(k,k-1)*X(k-1|k-1)        //控制量为0


2.计算预估计协方差矩阵
   P(k|k-1) = F(k,k-1)*P(k-1|k-1)*F(k,k-1)'+Q(k)
   Q(k) = U(k)×U(k)' 


3.计算卡尔曼增益矩阵
   Kg(k) = P(k|k-1)*H' / (H*P(k|k-1)*H' + R(k))
   R(k) = N(k)×N(k)' 


4.更新估计
   X(k|k) = X(k|k-1)+Kg(k)*(Z(k)-H*X(k|k-1))


5.计算更新后估计协防差矩阵
   P(k|k) =（I-Kg(k)*H）*P(k|k-1)


6. 更新最优值


F(k,k-1):     状态转移矩阵
X(k|k-1):     根据k-1时刻的最优值估计k时刻的值
X(k-1|k-1):   k-1时刻的最优值
P(k|k-1):     X(k|k-1)对应的covariance
P(k-1|k-1):   X(k-1|k-1)对应的covariance
Q(k):         系统过程的covariance
R(k):         测量过程的协方差
H(k):         观测矩阵转移矩阵
Z(k):         k时刻的测量值


基本思路: 首先根据上一次(如果是第一次则根据预赋值计算)的数据计算出本次的估计值,
          同理,根据上一次的数据计算出本次估计值的协方差;  接着,由本次估计值的协
          方差计算出卡尔曼增益;  最后,根据估测值和测量值计算当前最优值及其协方差
==============================================================================*/



//================================================//
//==             最优值方差结构体               ==//
//================================================//
typedef struct  _tCovariance
{
  float PNowOpt[LENGTH];
  float PPreOpt[LENGTH];
}tCovariance;



//================================================//
//==               最优值结构体                 ==//
//================================================//
typedef struct  _tOptimal
{
  float XNowOpt[LENGTH];
  float XPreOpt[LENGTH];
}tOptimal;



tOptimal      tOpt;
tCovariance   tCov;
float         Z[LENGTH]  = {4000};           //  测量值(每次测量的数据需要存入该数组)
float         I[LENGTH]  = {1};              //  单位矩阵
float         X[LENGTH]  = {0};              //  当前状态的预测值
float         P[LENGTH]  = {0};              //  当前状态的预测值的协方差
float         K[LENGTH]  = {0};              //  卡尔曼增益
float         Temp3[LENGTH] = {0};           //  辅助变量
//============================================================================//
//==                    卡尔曼滤波需要配置的变量                            ==//
//============================================================================//
float         F[LENGTH]  = {1};              //  状态转移矩阵
float         Q[LENGTH]  = {9};            //  系统过程的协方差
float         H[LENGTH]  = {1};              //  观测矩阵转移矩阵
float         R[LENGTH]  = {2};              //  测量过程的协方差
float         Temp1[LENGTH] = {1};           //  辅助变量, 同时保存tOpt.XPreOpt[]的初始化值
float         Temp2[LENGTH] = {10000};       //  辅助变量, 同时保存tCov.PPreOpt[]的初始化值





//============================================================================//
//==                          卡尔曼滤波                                    ==//
//============================================================================//
//==入口参数: 无                                                            ==//
//==出口参数: 无                                                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void KalMan(u16* in,u16* out)
{
  unsigned char   i;
//  unsigned short  k;
  
  for (i=0; i<LENGTH; i++)
  {
    tOpt.XPreOpt[i] = Temp1[i];           //零值初始化
  }
  for (i=0; i<LENGTH; i++)
  {
    tCov.PPreOpt[i] = Temp2[i];           //零值初始化
  }
  
  
//  for (k=0; k<N; k++)
//  {
    Z[0] = in[0];//(float)ADCSampling();
    MatrixMul(F, tOpt.XPreOpt, X, ORDER, ORDER, ORDER);       //  基于系统的上一状态而预测现在状态; X(k|k-1) = F(k,k-1)*X(k-1|k-1)
    
    MatrixCal(F, tCov.PPreOpt, Temp1, ORDER);
    MatrixAdd(Temp1, Q, P, ORDER, ORDER);                     //  预测数据的协方差矩阵; P(k|k-1) = F(k,k-1)*P(k-1|k-1)*F(k,k-1)'+Q
    
    MatrixCal(H, P, Temp1, ORDER);
    MatrixAdd(Temp1, R, Temp1, ORDER, ORDER);
    Gauss_Jordan(Temp1, ORDER);
    MatrixTrans(H, Temp2, ORDER, ORDER);
    MatrixMul(P, Temp2, Temp3, ORDER, ORDER, ORDER);
    MatrixMul(Temp1, Temp3, K, ORDER, ORDER, ORDER);          //  计算卡尔曼增益; Kg(k) = P(k|k-1)*H' / (H*P(k|k-1)*H' + R)
    
    MatrixMul(H, X, Temp1, ORDER, ORDER, ORDER);
    MatrixMinus(Z, Temp1, Temp1, ORDER, ORDER);
    MatrixMul(K, Temp1, Temp2, ORDER, ORDER, ORDER);
    MatrixAdd(X, Temp2, tOpt.XNowOpt, ORDER, ORDER);          //  根据估测值和测量值计算当前最优值; X(k|k) = X(k|k-1)+Kg(k)*(Z(k)-H*X(k|k-1))
    
    MatrixMul(K, H, Temp1, ORDER, ORDER, ORDER);
    MatrixMinus(I, Temp1, Temp1, ORDER, ORDER);
    MatrixMul(Temp1, P, tCov.PNowOpt, ORDER, ORDER, ORDER);   //  计算更新后估计协防差矩阵; P(k|k) =（I-Kg(k)*H）*P(k|k-1)
    
    for (i=0; i<LENGTH; i++)
    {
      tOpt.XPreOpt[i] = tOpt.XNowOpt[i];
      tCov.PPreOpt[i] = tCov.PNowOpt[i];
    }
		out[0]=(u16)(tOpt.XNowOpt[0]);
//  }
}





////============================================================================//
////==                    产生服从正态分布的随机数                            ==//
////============================================================================//
////==正态分布的概率密度函数  均值为u   方差为σ2(或标准差σ)                 ==//
////==                  Z=(x-μ)/σ ～ N(0,1)                                 ==//
////==入口参数: Num               产生随机数的个数                            ==//
////==          *S                保存随机数的数组                            ==//
////==          mu                需要的均值                                  ==//
////==          sigma             需要的方差                                  ==//
////==出口参数: *c                指向结果矩阵的指针                          ==//
////==返回值:   无                                                            ==//
////============================================================================//
//void Random(unsigned long Num, float *S, float mu, float sigma)
//{
//  unsigned long  j;
//  unsigned int   Temp;
//  float r;
//  
//  srand(SEED);                        //设置种子
//  
//  for(j=0; j<Num; j++)                //循环产生随机数
//  {
//    Temp = rand();
//    r = (Temp+0.00)/1073741823.00;
//    S[j] = sqrt(-2*log(r))*cos(2*3.14159265*rand())*sigma+mu;
//  }
//}

