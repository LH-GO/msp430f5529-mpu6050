/*
 * Filter.h
 *
 *  Created on: 2021��7��31��
 *      Author: LH
 */
#ifndef _Filter
#define _Filter

//***********�����ĺ궨��**************
#define Q_angle 0.001
#define Q_gyro 0.003
#define R_angle 0.5
#define dt 0.01     // dt��ȡֵΪkalman�˲�������ʱ��,�˴�Ϊ10ms;
#define C_0 1

//***********����ӿ�******************
//extern float Angle, Angle_dot; // �˲������Ǻ���Ǽ��ٶ�
extern void AngleCalcu(float angle_m,float gyro_m,float *p); // 

#endif
