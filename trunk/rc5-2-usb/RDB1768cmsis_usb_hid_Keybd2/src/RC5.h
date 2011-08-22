/*
 * RC5.h
 *
 *  Created on: 14/08/2011
 *      Author: ARCESCH
 */

#ifndef RC5_H_
#define RC5_H_

//<-- RC5
#include "lpc17xx_gpio.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"

void RC5_Shift_Bit(char);
void RC5_Decode(void);
void RC5_Init(void);
void TIMER0_IRQHandler(void);

#define CAMBIO 1

#define RC5_INPUT_PIN      (1<<26)		   // Pongo como entrada P1.26

#define MIN_HALF_BIT       640             // 640 us
#define HALF_BIT_TIME      889             // 889 us
#define MAX_HALF_BIT       1140            // 1140 us
#define MIN_FULL_BIT       1340            // 1340 us
#define FULL_BIT_TIME      1778            // 1778 us
#define MAX_FULL_BIT       2220            // 2220 us

#define RETARDO			   0               //

unsigned char RC5_System;                  // Format 1 E/N t  s4 s3 s3 s1 s0
unsigned char RC5_Command;                 // Format 0  0  c5 c4 c3 c2 c1 c0
unsigned char RC5_flag;

static signed int    low_time;
static signed int    high_time;
static unsigned char half_bit;
static unsigned char sys;                  // system byte
static unsigned char cmd;                  // Command byte





/* Interrupt service routines */
void TIMER0_IRQHandler(void);
void TIMER1_IRQHandler(void);
//int a;
//int b=1;

// RC5 -->


#endif /* RC5_H_ */
