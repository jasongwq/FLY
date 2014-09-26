//=====================================================================================================
// IMU.c
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer
// units are irrelevant as the vector is normalised.
//
//=====================================================================================================
//----------------------------------------------------------------------------------------------------
// Header files
#include "IMU.h"
#include <math.h>
//----------------------------------------------------------------------------------------------------
// Definitions
#define Kp 10.0f       // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f     // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f  // half the sample period(s)

#define Gyro_Gr     0.0010653f
//---------------------------------------------------------------------------------------------------
// Variable definitions
float q0 = 1, q1 = 0, q2 = 0, q3 = 0; // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;// scaled integral error
//====================================================================================================
// Function
//====================================================================================================
T_float_angle   Att_Angle;  //ATT函数计算出的姿态角
T_float_angle   Att_Angle_Avg;  //ATT函数计算出的姿态角

float compass_yaw;
void IMUupdate(S_INT16_XYZ *gyr, S_INT16_XYZ *acc, T_float_angle *angle)
{
    float ax = acc->x, ay = acc->y, az = acc->z;
    float gx = gyr->x, gy = gyr->y, gz = gyr->z;
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    gx *= Gyro_Gr;
    gy *= Gyro_Gr;
    gz *= Gyro_Gr;
    // normalise the measurements
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    // 把加计的三维向量转成单位向量。
    // estimated direction of gravity
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    // 这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
    // 根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
    // 所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
    // error is sum of cross product between reference direction of field and direction measured by sensor
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    // axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
    // axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
    // 那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
    // 向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
    // 这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
    // integral error scaled integral gain
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;
    // adjusted gyroscope measurements
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
    // 用叉积误差来做PI修正陀螺零偏
    // integrate quaternion rate and normalise
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    // 四元数微分方程
    // normalise quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    // 四元数规范化
    //angle->yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
    angle->pit = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // pitch
    angle->rol = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll
    //四元数转化成欧拉角

}
////====================================================================================================
//// END OF CODE
////====================================================================================================
////////////////////////////////////////////////////////////////////////////////





//#define Kp 10.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.008f                          // integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.001f                   // half the sample period???????

//float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
//float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
//void IMUupdate(S_INT16_XYZ *gyr, S_INT16_XYZ *acc, T_float_angle *angle)
//{
//  float ax = acc->X,ay = acc->Y,az = acc->Z;
//  float gx = gyr->X,gy = gyr->Y,gz = gyr->Z;
//  float norm;
////  float hx, hy, hz, bx, bz;
//  float vx, vy, vz;// wx, wy, wz;
//  float ex, ey, ez;

//  // ???????????
//  float q0q0 = q0*q0;
//  float q0q1 = q0*q1;
//  float q0q2 = q0*q2;
////  float q0q3 = q0*q3;
//  float q1q1 = q1*q1;
////  float q1q2 = q1*q2;
//  float q1q3 = q1*q3;
//  float q2q2 = q2*q2;
//  float q2q3 = q2*q3;
//  float q3q3 = q3*q3;
//
//  if(ax*ay*az==0)
//      return;
//
//  gx *= Gyro_Gr;
//  gy *= Gyro_Gr;
//  gz *= Gyro_Gr;
//
//  norm = sqrt(ax*ax + ay*ay + az*az);       //acc?????
//  ax = ax /norm;
//  ay = ay / norm;
//  az = az / norm;

//  // estimated direction of gravity and flux (v and w)              ?????????/??
//  vx = 2*(q1q3 - q0q2);                                               //????xyz???
//  vy = 2*(q0q1 + q2q3);
//  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

//  // error is sum of cross product between reference direction of fields and direction measured by sensors
//  ex = (ay*vz - az*vy) ;                                               //???????????????
//  ey = (az*vx - ax*vz) ;
//  ez = (ax*vy - ay*vx) ;

//  exInt = exInt + ex * Ki;                                  //???????
//  eyInt = eyInt + ey * Ki;
//  ezInt = ezInt + ez * Ki;

//  // adjusted gyroscope measurements
//  gx = gx + Kp*ex + exInt;                                                //???PI???????,???????
//  gy = gy + Kp*ey + eyInt;
//  gz = gz + Kp*ez + ezInt;                                            //???gz????????????????,??????????????

//  // integrate quaternion rate and normalise                         //????????
//  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

//  // normalise quaternion
//  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//
//  q0 = q0 / norm;
//  q1 = q1 / norm;
//  q2 = q2 / norm;
//  q3 = q3 / norm;

////  Q_ANGLE.YAW = GYRO_I.Z;//atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
//  angle->pit = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 ; // pitch
//  angle->rol = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
//}
