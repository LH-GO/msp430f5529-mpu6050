/*
 * Filter.c
 *
 *  Created on: 2021年7月31日
 *      Author: LH
 */
/*
***************************************************************************
**文件名：Filter.c
**编写者：zlstone
**描  述：Kalman滤波，8MHz的处理时间约1.8ms；
****************************************************************************
*/
#include "Filter.h"

static float Angle, Angle_dot;   //外部需要引用的变量,倾角值和倾角加速度值

//*************内部变量定义区************************
static float P[2][2] = {
                { 1, 0 },
                { 0, 1 }
               };

static float Pdot[4] ={0,0,0,0};

static float Q_bias, Angle_err;
static float PCt_0, PCt_1, E;
static float K_0, K_1, t_0, t_1;



/*
**********************************************
**函数名  ：void Kalman_Filter(float angle_m,float gyro_m)
**函数功能：输出陀螺仪测量的倾角加速度
**返回参数：无
**传入参数：angle_m - 倾角测量值
**          GYRO_XOUT - 倾角加速度测量值
**********************************************
*/
void Kalman_Filter(float angle_m,float gyro_m)  //gyro_m:gyro_measure
{
    Angle += (gyro_m-Q_bias) * dt;  //先验估计

    Pdot[0]=Q_angle - P[0][1] - P[1][0]; // Pk-先验估计误差协方差的微分
    Pdot[1]= - P[1][1];
    Pdot[2]= - P[1][1];
    Pdot[3]=Q_gyro;

    P[0][0] += Pdot[0] * dt;  // Pk-先验估计误差协方差微分的积分
    P[0][1] += Pdot[1] * dt;  // 先验估计误差协方差
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    Angle_err = angle_m - Angle;  //zk-先验估计

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;  //后验估计误差协方差
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;


    Angle   += K_0 * Angle_err; //后验估计
    Q_bias  += K_1 * Angle_err; //后验估计
    Angle_dot = gyro_m-Q_bias;  //输出值(后验估计)的微分=角速度
}

/*
**********************************************
**函数名  ：void AngleCalcu(float angle_m,float gyro_m)
**函数功能：互补滤波
**            补偿原理是取当前倾角和加速度获得倾角差值进行放大，然后与
**            陀螺仪角速度叠加后再积分，从而使倾角最跟踪为加速度获得的角度
**            R_angle为放大倍数，可调节补偿度;dt为系统周期10ms
**返回参数：无
**传入参数：angle_m - 倾角测量值
**          GYRO_XOUT - 倾角加速度测量值
**          *p - 指向转换后得到的倾角和倾角加速度存储数组
**********************************************
*/
void AngleCalcu(float angle_m,float gyro_m,float *p)
{
    //float Angle_sum; // 测量角度运算暂存变量
    //*************卡尔曼滤波融合***************
    Kalman_Filter(angle_m,gyro_m);       //卡尔曼滤波计算倾角
    
    //*************互补滤波****************
    Angle = Angle + (((angle_m - Angle)*R_angle + gyro_m)*dt);
    
    p[0] = Angle; // 第一位存储倾角值
    p[1] = Angle_dot; // 第二位存储倾角加速度值
}
