/*
 * Filter.h
 *
 *  Created on: 2021年7月31日
 *      Author: LH
 */
#ifndef _Filter
#define _Filter

//***********变量的宏定义**************
#define Q_angle 0.001
#define Q_gyro 0.003
#define R_angle 0.5
#define dt 0.01     // dt的取值为kalman滤波器采样时间,此处为10ms;
#define C_0 1

//***********输出接口******************
//extern float Angle, Angle_dot; // 滤波后的倾角和倾角加速度
extern void AngleCalcu(float angle_m,float gyro_m,float *p); // 

#endif
