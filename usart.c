/*
 * usart.c
 *
 *  Created on: 2021年7月31日
 *      Author: LH
 */
#include "usart.h"
#include <msp430.h>

void Usart_Init(void)
{
    P3SEL = BIT3+BIT4;                        // P3.4,5 = USCI_A0 TXD/RXD复用功能
    UCA0CTL1 |= UCSWRST|UCSSEL_2;             //UCA0CTL0默认,UCA0CTL1设置选择时钟等标志
    UCA0CTL1 &= ~UCSWRST;                     //0:USCI重置已释放，以便运行

    UCA0BR0 = UCAXBR0;                              // 1.48576MHz 9600 查表36.5
    UCA0BR1 = UCAXBR1;                              // 1.48576MHz 9600

    UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // UCBRSx=0,不懂这个UCBRF_13为啥要长13个Bk,开启过采样模式
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt接收中断使能
}


void senfchar(char s)                                   //发送字符
{
    UCA0TXBUF=s;
    while(!(UCA0IFG&UCTXIFG));
}

void sendstring(unsigned char *p)                       //发送字符串
{
    while(*p!='\0')
    {
        while(!(UCA0IFG&UCTXIFG));
        UCA0TXBUF=*p++;

    }
}

// Echo back RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
                                              //中断查询__even_in_range(矢量寄存器，偶数)
  switch(__even_in_range(UCA0IV,4))           //只有在UCA0IV的值是在0-4内的偶数时才会执行switch函数内的语句
  {
    case 0:break;                             // Vector 0 - no interrupt请求
    case 2:                                   // Vector 2 - RXIFG具有最高优先级
    while (!(UCA0IFG&UCTXIFG));               // USCI_A0 TX buffer ready?
    UCA0TXBUF = UCA0RXBUF;                    // TX -> RXed character
    break;
    case 4:break;                             // Vector 4 - TXIFG发送不为空标志，具有最低优先级
    default: break;
  }
  UCA0IFG &=~(2<<0);                        //清空接收和发送中断请求标志
}
