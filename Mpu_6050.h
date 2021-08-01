/*
 * Mpu6050.h
 *
 *  Created on: 2021��7��31��
 *      Author: LH
 */
#include "mytype.h"
#ifndef Mpu_6050
#define Mpu_6050

//********Mpu6050�����У׼ֵ**************
#define MPU6050_ZERO_ACCELL 378
#define MPU6050_ZERO_GYRO 13

//****����Mpu6050Ӳ���ӿڣ���ͬӲ���޸Ĵ˴����ɣ�***********
#define MPU6050DIR P1DIR
#define MPU6050OUT P1OUT
#define MPU6050IN  P1IN
#define MPU_SCL BIT2
#define MPU_SDA BIT3

//*********I2CӲ����ؽӿں���**************
#define MPU_SCL_OUT() MPU6050DIR |= MPU_SCL  // SCL�����
#define MPU_SCL_H()   MPU6050OUT |= MPU_SCL  // SCL����
#define MPU_SCL_L()   MPU6050OUT &= ~MPU_SCL // SCL����
#define MPU_SDA_OUT() MPU6050DIR |= MPU_SDA  // SDA�����
#define MPU_SDA_H()   MPU6050OUT |= MPU_SDA  // SDA����
#define MPU_SDA_L()   MPU6050OUT &= ~MPU_SDA // SDA����
#define MPU_SDA_IN()  MPU6050DIR &= ~MPU_SDA // SDA������
#define MPU_SDA_DAT() (MPU6050IN&MPU_SDA)  // SDA��������

//*************����MPU6050�ڲ���ַ*******************
#define SMPLRT_DIV      0x19    //�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define CONFIG          0x1A    //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define GYRO_CONFIG     0x1B    //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define ACCEL_CONFIG            0x1C    //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
/***************���ٶȴ������Ĵ���******************/
#define ACCEL_XOUT_H            0x3B
#define ACCEL_XOUT_L            0x3C
#define ACCEL_XOUT      ACCEL_XOUT_H    // X���ȡ��ַ����λΪ��ʼλ
#define ACCEL_YOUT_H            0x3D
#define ACCEL_YOUT_L            0x3E
#define ACCEL_YOUT      ACCEL_YOUT_H    // Y���ȡ��ַ����λΪ��ʼλ
#define ACCEL_ZOUT_H            0x3F
#define ACCEL_ZOUT_L            0x40
#define ACCEL_ZOUT      ACCEL_ZOUT_H    // Z���ȡ��ַ����λΪ��ʼλ
/*****************�¶ȴ������Ĵ���****************/
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define TEMP_OUT          TEMP_OUT_H    // �¶ȴ�������ȡ��ַ����λΪ��ʼλ
/*****************�����ǼĴ���********************/
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_XOUT        GYRO_XOUT_H    // ������X���ȡ��ַ����λΪ��ʼλ
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_YOUT        GYRO_YOUT_H    // ������Y���ȡ��ַ����λΪ��ʼλ
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48
#define GYRO_ZOUT        GYRO_ZOUT_H    // ������Z���ȡ��ַ����λΪ��ʼλ

#define PWR_MGMT_1      0x6B    //��Դ��������ֵ��0x00(��������)
#define WHO_AM_I            0x75    //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define SlaveAddress            0xD0    //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ

//********************��ʱ�ӿڳ���**********************
#define CPU_F ((double)8000000) //8MHZ ���񣨴˴�Ӧ������CPUƵ����ͬ��
#define DELAY_US(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0))  // MS����ʱ����
#define DELAY_MS(x) __delay_cycles((long)(CPU_F*(double)x/1000.0))   // US����ʱ����

//**************����ӿں���***********************
extern void MpuClockInit();
extern void ByteWrite6050(uchar REG_Address,uchar REG_data);
extern uchar ByteRead6050(uchar REG_Address);
extern int Get6050Data(uchar REG_Address);
extern void InitMPU6050();
extern float Mpu6050AccelAngle(int8 dir);
extern float Mpu6050GyroAngle(int8 dir);

#endif
