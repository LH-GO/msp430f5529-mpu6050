/*
 * mpu6050.c
 *
 *  Created on: 2021年7月31日
 *      Author: LH
 */
#include "msp430.h"
#include "mytype.h"
#include "Mpu_6050.h"

static void I2C_Start();
static void I2C_Stop();
static void I2C_SendACK(uchar ack);
static uchar I2C_RecvACK();
static void I2C_SendByte(uchar dat);
static uchar I2C_RecvACK();

//**************************************
//I2C起始信号
//**************************************
void I2C_Start()
{
    MPU_SCL_OUT(); // SCL设置为输出
    MPU_SDA_OUT(); // SDA设置为输出
    
    MPU_SDA_H();   //拉高数据线
    MPU_SCL_H();   //拉高时钟线
    DELAY_US(5);   //延时
    MPU_SDA_L();   //产生下降沿
    DELAY_US(5);   //延时
    MPU_SCL_L();   //拉低时钟线
}
//**************************************
//I2C停止信号
//**************************************
void I2C_Stop()
{
    MPU_SCL_OUT(); // SCL设置为输出
    MPU_SDA_OUT(); // SDA设置为输出
    
    MPU_SDA_L();   //拉低数据线
    MPU_SCL_H();   //拉高时钟线
    DELAY_US(5);   //延时
    MPU_SDA_H();   //产生上升沿
    DELAY_US(5);   //延时
}
//**************************************
//I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(uchar ack)
{
    MPU_SCL_OUT(); // SCL设置为输出
    MPU_SDA_OUT(); // SDA设置为输出
    
    if(ack) 
        MPU_SDA_H();
    else
        MPU_SDA_L();
//    SDA = ack;                  //写应答信号
    MPU_SCL_H();                    //拉高时钟线
    DELAY_US(5);                  //延时
    MPU_SCL_L();                    //拉低时钟线
    DELAY_US(5);                  //延时
}
//**************************************
//I2C接收应答信号
//**************************************
uchar I2C_RecvACK()
{
    uchar cy;
    MPU_SCL_OUT(); // SCL设置为输出
    MPU_SDA_IN(); // SDA设置为输入
    
    MPU_SCL_H();                    //拉高时钟线
    DELAY_US(5);                 //延时
    if(MPU_SDA_DAT())
    {
        cy=1;
    }
    else 
    {
      cy=0;
    }
//    cy = SDA;                   //读应答信号
    MPU_SCL_L();                    //拉低时钟线
    DELAY_US(5);                //延时
    MPU_SDA_OUT(); // SDA设置为输出
    
    return cy;
    
}
//**************************************
//向I2C总线发送一个字节数据
//**************************************
void I2C_SendByte(uchar dat)
{
    uchar i;
    MPU_SCL_OUT(); // SCL设置为输出
    MPU_SDA_OUT(); // SDA设置为输出
    for (i=0; i<8; i++)         //8位计数器
    {
        if((dat<<i)&0x80)
        {
            MPU_SDA_H();
        }
        else 
        {
            MPU_SDA_L();
        }
       // SDA = cy;               //送数据口
        MPU_SCL_H();                //拉高时钟线
        DELAY_US(5);              //延时
        MPU_SCL_L();                //拉低时钟线
        DELAY_US(5);              //延时
    }
    I2C_RecvACK();
}
//**************************************
//从I2C总线接收一个字节数据
//**************************************
uchar I2C_RecvByte()
{
    uchar i;
    uchar dat = 0,cy;
    MPU_SCL_OUT(); // SCL设置为输出
    MPU_SDA_OUT(); // SDA设置为输出
    
    MPU_SDA_H();  //使能内部上拉,准备读取数据,
    MPU_SDA_IN(); // SDA设置为输入，准备向主机输入数据
    for (i=0; i<8; i++)         //8位计数器
    {
       
        dat <<= 1;
        MPU_SCL_H();                //拉高时钟线
        DELAY_US(5);             //延时
        if(MPU_SDA_DAT()) 
        {
            cy=1;
        }
        else 
        {
          cy=0;
        }
        dat |= cy;             //读数据
        MPU_SCL_L();                //拉低时钟线
        DELAY_US(5);             //延时
    }
    MPU_SDA_OUT();
    return dat;
}
//**************************************
//向I2C设备写入一个字节数据
//**************************************
void ByteWrite6050(uchar REG_Address,uchar REG_data)
{
    I2C_Start();                  //起始信号
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号
    I2C_SendByte(REG_Address);    //内部寄存器地址，
    I2C_SendByte(REG_data);       //内部寄存器数据，
    I2C_Stop();                   //发送停止信号
}
//**************************************
//从I2C设备读取一个字节数据
//**************************************
uchar ByteRead6050(uchar REG_Address)
{
    uchar REG_data;
    I2C_Start();                   //起始信号
    I2C_SendByte(SlaveAddress);    //发送设备地址+写信号
    I2C_SendByte(REG_Address);     //发送存储单元地址，从0开始
    I2C_Start();                   //起始信号
    I2C_SendByte(SlaveAddress+1);  //发送设备地址+读信号
    REG_data=I2C_RecvByte();       //读出寄存器数据
    I2C_SendACK(1);                //接收应答信号
    I2C_Stop();                    //停止信号
    return REG_data;
}
//**************************************
//合成数据
//**************************************
int Get6050Data(uchar REG_Address)
{
    char H,L;
    H=ByteRead6050(REG_Address);
    L=ByteRead6050(REG_Address+1);
    return (H<<8)+L;   //合成数据
}
//**************************************
//初始化MPU6050
//**************************************
void InitMPU6050()
{
    ByteWrite6050(PWR_MGMT_1, 0x00);  // 解除休眠状态
    ByteWrite6050(SMPLRT_DIV, 0x07);  // 陀螺仪采样率设置（125HZ）
    ByteWrite6050(CONFIG, 0x06);      // 低通滤波器频率设置（5HZ）
    ByteWrite6050(GYRO_CONFIG, 0x18); // 陀螺仪自检及检测范围设置(不自检,16.4LSB/DBS/S)
    ByteWrite6050(ACCEL_CONFIG, 0x01); // 加速计自检、测量范围及高通滤波频率(不自检，2G(16384LSB/G)，5Hz)
}

/*
**********************************************
**函数名  ：float Mpu6050AccelAngle(int8 dir)
**函数功能：输出加速度传感器测量的倾角值
**            范围为2g时，换算关系：16384 LSB/g
**            角度较小时，x=sinx得到角度（弧度）, deg = rad*180/3.14
**            因为x>=sinx,故乘以1.2适当放大
**返回参数：测量的倾角值
**传入参数：dir - 需要测量的方向
**           ACCEL_XOUT - X方向
**           ACCEL_YOUT - Y方向
**           ACCEL_ZOUT - Z方向
**********************************************
*/
float Mpu6050AccelAngle(int8 dir)
{
    float accel_agle; // 测量的倾角值
    float result; // 测量值缓存变量
    result = (float)Get6050Data(dir); // 测量当前方向的加速度值,转换为浮点数
    accel_agle = (result + MPU6050_ZERO_ACCELL)/16384; // 去除零点偏移,计算得到角度（弧度）
    accel_agle = accel_agle*1.2*180/3.14;     //弧度转换为度
    
    return accel_agle; // 返回测量值
}

/*
**********************************************
**函数名  ：float Mpu6050GyroAngle(int8 dir)
**函数功能：输出陀螺仪测量的倾角加速度
**            范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)
**返回参数：测量的倾角加速度值
**传入参数：dir - 需要测量的方向
**           GYRO_XOUT - X轴方向
**           GYRO_YOUT - Y轴方向
**           GYRO_ZOUT - Z轴方向
**********************************************
*/
float Mpu6050GyroAngle(int8 dir)
{
    float gyro_angle;
    gyro_angle = (float)Get6050Data(dir);   // 检测陀螺仪的当前值
    gyro_angle = -(gyro_angle + MPU6050_ZERO_GYRO)/16.4;    //去除零点偏移，计算角速度值,负号为方向处理
    
    return gyro_angle; // 返回测量值
}

