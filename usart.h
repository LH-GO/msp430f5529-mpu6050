/*
 * usart.h
 *
 *  Created on: 2021��7��31��
 *      Author: LH
 */
#ifndef __USART_H
#define __USART_H

/*************************************
     P3.4/UCARXT ----��---TXD
     P3.5/UCATXT ----��---RXD
     9600���С��
**************************************/
#include <msp430.h>

#define CPU_CLOCK       1000000
#define delay_us(us)    __delay_cycles(CPU_CLOCK/1000000*(us))
#define delay_ms(ms)    __delay_cycles(CPU_CLOCK/1000*(ms))

#define u16 unsigned int
#define u8  unsigned char

#define  UCAXBR0         6                  //9600  ������
#define  UCAXBR1         0



void Usart_Init(void);
void senfchar(char s);
void sendstring(unsigned char *p);


#endif
