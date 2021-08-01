/*
 * Filter.c
 *
 *  Created on: 2021��7��31��
 *      Author: LH
 */
/*
***************************************************************************
**�ļ�����Filter.c
**��д�ߣ�zlstone
**��  ����Kalman�˲���8MHz�Ĵ���ʱ��Լ1.8ms��
****************************************************************************
*/
#include "Filter.h"

static float Angle, Angle_dot;   //�ⲿ��Ҫ���õı���,���ֵ����Ǽ��ٶ�ֵ

//*************�ڲ�����������************************
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
**������  ��void Kalman_Filter(float angle_m,float gyro_m)
**�������ܣ���������ǲ�������Ǽ��ٶ�
**���ز�������
**���������angle_m - ��ǲ���ֵ
**          GYRO_XOUT - ��Ǽ��ٶȲ���ֵ
**********************************************
*/
void Kalman_Filter(float angle_m,float gyro_m)  //gyro_m:gyro_measure
{
    Angle += (gyro_m-Q_bias) * dt;  //�������

    Pdot[0]=Q_angle - P[0][1] - P[1][0]; // Pk-����������Э�����΢��
    Pdot[1]= - P[1][1];
    Pdot[2]= - P[1][1];
    Pdot[3]=Q_gyro;

    P[0][0] += Pdot[0] * dt;  // Pk-����������Э����΢�ֵĻ���
    P[0][1] += Pdot[1] * dt;  // ����������Э����
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    Angle_err = angle_m - Angle;  //zk-�������

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;  //����������Э����
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;


    Angle   += K_0 * Angle_err; //�������
    Q_bias  += K_1 * Angle_err; //�������
    Angle_dot = gyro_m-Q_bias;  //���ֵ(�������)��΢��=���ٶ�
}

/*
**********************************************
**������  ��void AngleCalcu(float angle_m,float gyro_m)
**�������ܣ������˲�
**            ����ԭ����ȡ��ǰ��Ǻͼ��ٶȻ����ǲ�ֵ���зŴ�Ȼ����
**            �����ǽ��ٶȵ��Ӻ��ٻ��֣��Ӷ�ʹ��������Ϊ���ٶȻ�õĽǶ�
**            R_angleΪ�Ŵ������ɵ��ڲ�����;dtΪϵͳ����10ms
**���ز�������
**���������angle_m - ��ǲ���ֵ
**          GYRO_XOUT - ��Ǽ��ٶȲ���ֵ
**          *p - ָ��ת����õ�����Ǻ���Ǽ��ٶȴ洢����
**********************************************
*/
void AngleCalcu(float angle_m,float gyro_m,float *p)
{
    //float Angle_sum; // �����Ƕ������ݴ����
    //*************�������˲��ں�***************
    Kalman_Filter(angle_m,gyro_m);       //�������˲��������
    
    //*************�����˲�****************
    Angle = Angle + (((angle_m - Angle)*R_angle + gyro_m)*dt);
    
    p[0] = Angle; // ��һλ�洢���ֵ
    p[1] = Angle_dot; // �ڶ�λ�洢��Ǽ��ٶ�ֵ
}
