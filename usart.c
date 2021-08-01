/*
 * usart.c
 *
 *  Created on: 2021��7��31��
 *      Author: LH
 */
#include "usart.h"
#include <msp430.h>

void Usart_Init(void)
{
    P3SEL = BIT3+BIT4;                        // P3.4,5 = USCI_A0 TXD/RXD���ù���
    UCA0CTL1 |= UCSWRST|UCSSEL_2;             //UCA0CTL0Ĭ��,UCA0CTL1����ѡ��ʱ�ӵȱ�־
    UCA0CTL1 &= ~UCSWRST;                     //0:USCI�������ͷţ��Ա�����

    UCA0BR0 = UCAXBR0;                              // 1.48576MHz 9600 ���36.5
    UCA0BR1 = UCAXBR1;                              // 1.48576MHz 9600

    UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // UCBRSx=0,�������UCBRF_13ΪɶҪ��13��Bk,����������ģʽ
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt�����ж�ʹ��
}


void senfchar(char s)                                   //�����ַ�
{
    UCA0TXBUF=s;
    while(!(UCA0IFG&UCTXIFG));
}

void sendstring(unsigned char *p)                       //�����ַ���
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
                                              //�жϲ�ѯ__even_in_range(ʸ���Ĵ�����ż��)
  switch(__even_in_range(UCA0IV,4))           //ֻ����UCA0IV��ֵ����0-4�ڵ�ż��ʱ�Ż�ִ��switch�����ڵ����
  {
    case 0:break;                             // Vector 0 - no interrupt����
    case 2:                                   // Vector 2 - RXIFG����������ȼ�
    while (!(UCA0IFG&UCTXIFG));               // USCI_A0 TX buffer ready?
    UCA0TXBUF = UCA0RXBUF;                    // TX -> RXed character
    break;
    case 4:break;                             // Vector 4 - TXIFG���Ͳ�Ϊ�ձ�־������������ȼ�
    default: break;
  }
  UCA0IFG &=~(2<<0);                        //��ս��պͷ����ж������־
}
