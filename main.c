#include <msp430.h> 
#include "mytype.h"
#include "Mpu_6050.h"
#include "Filter.h"
#include "usart.h"

unsigned char a[3]={'a','b','c'};


/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	int Angle1=0;
	float result_accel,result_gyro; // 转换结果缓存
	float filter_result[2]; // 缓存滤波后的倾角值和倾角加速度值
	char sum1[10],sum2[10]; // 串口发送缓存
	InitMPU6050(); // 初始化模块
	Usart_Init();
	DELAY_MS(200);

	while(1)
	{
//	    DELAY_MS(1);

        result_accel = Mpu6050AccelAngle(ACCEL_YOUT); // 测量加速度传感器的Y轴值
        result_gyro = Mpu6050GyroAngle(GYRO_XOUT); // 测量陀螺仪的X轴值
        AngleCalcu(result_accel,result_gyro,filter_result);// 卡尔曼滤波及融合
        Angle1=filter_result[0];
        if(Angle1<0)
        {
            Angle1=-Angle1;
            sum1[0]='-';sum1[1]='0'+Angle1/100;sum1[2]='0'+Angle1%100/10;sum1[3]='0'+Angle1%10;sum1[4]='\n';
        }
        else
        {
            sum1[0]='0'+Angle1/100;sum1[1]='0'+Angle1%100/10;sum1[2]='0'+Angle1%10;sum1[3]='\n';sum1[4]='\0';
        }
        //result_gyro = (float)Get6050Data(GYRO_XOUT);
//        sprintf(sum1,"%.2f",filter_result[0]); // 将测量倾角值转换为字符串
//        sprintf(sum2,"%.2f",filter_result[1]); // 将测量倾角加速度值转换为字符串

	    sendstring(sum1);
//        SerPrint_Str(a);
//        SerPrint_Str(sum1);
//        SerPrint_Char(' ');
//        SerPrint_Str("The measured Gyro is : ");
//        SerPrint_Str(sum2);
//        SerPrint_Char('\n');

	}

	return 0;
}
