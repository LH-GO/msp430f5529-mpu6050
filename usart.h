/*
 * usart.h
 *
 *  Created on: 2021年7月31日
 *      Author: LH
 */
#ifndef __USART_H
#define __USART_H

/*************************************
     P3.4/UCARXT ----接---TXD
     P3.5/UCATXT ----接---RXD
     9600误差小点
**************************************/
#include <msp430.h>

#define CPU_CLOCK       1000000
#define delay_us(us)    __delay_cycles(CPU_CLOCK/1000000*(us))
#define delay_ms(ms)    __delay_cycles(CPU_CLOCK/1000*(ms))

#define u16 unsigned int
#define u8  unsigned char

#define  UCAXBR0         6                  //9600  波特率
#define  UCAXBR1         0



void Usart_Init(void);
void senfchar(char s);
void sendstring(unsigned char *p);


#endif
