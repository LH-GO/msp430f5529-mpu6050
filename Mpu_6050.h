/*
 * Mpu6050.h
 *
 *  Created on: 2021年7月31日
 *      Author: LH
 */
#include "mytype.h"
#ifndef Mpu_6050
#define Mpu_6050

//********Mpu6050的零点校准值**************
#define MPU6050_ZERO_ACCELL 378
#define MPU6050_ZERO_GYRO 13

//****定义Mpu6050硬件接口（不同硬件修改此处即可）***********
#define MPU6050DIR P1DIR
#define MPU6050OUT P1OUT
#define MPU6050IN  P1IN
#define MPU_SCL BIT2
#define MPU_SDA BIT3

//*********I2C硬件相关接口函数**************
#define MPU_SCL_OUT() MPU6050DIR |= MPU_SCL  // SCL脚输出
#define MPU_SCL_H()   MPU6050OUT |= MPU_SCL  // SCL拉高
#define MPU_SCL_L()   MPU6050OUT &= ~MPU_SCL // SCL拉低
#define MPU_SDA_OUT() MPU6050DIR |= MPU_SDA  // SDA脚输出
#define MPU_SDA_H()   MPU6050OUT |= MPU_SDA  // SDA拉高
#define MPU_SDA_L()   MPU6050OUT &= ~MPU_SDA // SDA拉低
#define MPU_SDA_IN()  MPU6050DIR &= ~MPU_SDA // SDA脚输入
#define MPU_SDA_DAT() (MPU6050IN&MPU_SDA)  // SDA输入数据

//*************定义MPU6050内部地址*******************
#define SMPLRT_DIV      0x19    //陀螺仪采样率，典型值：0x07(125Hz)
#define CONFIG          0x1A    //低通滤波频率，典型值：0x06(5Hz)
#define GYRO_CONFIG     0x1B    //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define ACCEL_CONFIG            0x1C    //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
/***************加速度传感器寄存器******************/
#define ACCEL_XOUT_H            0x3B
#define ACCEL_XOUT_L            0x3C
#define ACCEL_XOUT      ACCEL_XOUT_H    // X轴读取地址，高位为起始位
#define ACCEL_YOUT_H            0x3D
#define ACCEL_YOUT_L            0x3E
#define ACCEL_YOUT      ACCEL_YOUT_H    // Y轴读取地址，高位为起始位
#define ACCEL_ZOUT_H            0x3F
#define ACCEL_ZOUT_L            0x40
#define ACCEL_ZOUT      ACCEL_ZOUT_H    // Z轴读取地址，高位为起始位
/*****************温度传感器寄存器****************/
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define TEMP_OUT          TEMP_OUT_H    // 温度传感器读取地址，高位为起始位
/*****************陀螺仪寄存器********************/
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_XOUT        GYRO_XOUT_H    // 陀螺仪X轴读取地址，高位为起始位
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_YOUT        GYRO_YOUT_H    // 陀螺仪Y轴读取地址，高位为起始位
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48
#define GYRO_ZOUT        GYRO_ZOUT_H    // 陀螺仪Z轴读取地址，高位为起始位

#define PWR_MGMT_1      0x6B    //电源管理，典型值：0x00(正常启用)
#define WHO_AM_I            0x75    //IIC地址寄存器(默认数值0x68，只读)
#define SlaveAddress            0xD0    //IIC写入时的地址字节数据，+1为读取

//********************延时接口程序**********************
#define CPU_F ((double)8000000) //8MHZ 晶振（此处应该与其CPU频率相同）
#define DELAY_US(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0))  // MS级延时函数
#define DELAY_MS(x) __delay_cycles((long)(CPU_F*(double)x/1000.0))   // US级延时函数

//**************对外接口函数***********************
extern void MpuClockInit();
extern void ByteWrite6050(uchar REG_Address,uchar REG_data);
extern uchar ByteRead6050(uchar REG_Address);
extern int Get6050Data(uchar REG_Address);
extern void InitMPU6050();
extern float Mpu6050AccelAngle(int8 dir);
extern float Mpu6050GyroAngle(int8 dir);

#endif
