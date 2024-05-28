// Timer0.c
// Runs on LM4F120/TM4C123
// Use TIMER0 in 32-bit periodic mode to request interrupts at a periodic rate
// Daniel Valvano
// March 20, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
  Program 7.5, example 7.6

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"
#define oneus 16


void (*PeriodicTask)(void);   // user function
static volatile unsigned long int TIME=0;
unsigned long long int period = 0xFFFF;


// ***************** Timer0_Init ****************
// Activate TIMER0 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
void Timer1A_Init(){
  SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER0
  TIMER1_CFG_R = 0x00000004;    // 2) configure for 16-bit mode
  TIMER1_TAMR_R = 0x00000002;   // 3) configure for periodic down-count mode
  TIMER1_TAILR_R = period;    // 4) max reload value
  TIMER1_TAPR_R = 0;            // 5) bus clock prescale
  TIMER1_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
  NVIC_PRI5_R = (NVIC_PRI4_R&0xFFFF1FFF)|0x00006000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 35, interrupt number 19
  NVIC_EN0_R = 1<<21;           // 9) enable IRQ 19 in NVIC
  TIMER1_CTL_R = 0x00000001;    // 10) enable TIMER1A
}


void GPTM_Wait1us(int delay){
  TIMER1_CTL_R = 0x00000000; //disable timer
  TIMER1_TAILR_R = delay*oneus - 1; // number of counts to wait
  TIMER1_CTL_R = 0x00000001; 
  while((TIMER1_RIS_R&0x01)==0); // wait for count flag
  TIMER1_CTL_R = 0x00000000;
}




uint32_t GPTM_Get_MC_Elapsed(void){
		TIME = (period)-TIMER1_TAR_R;
    return (period)-TIMER1_TAR_R;
}

void Timer1A_Start(void){
  TIMER1_CTL_R = 0x00000000;
  TIMER1_TAILR_R = period; // number of counts to wait                 
    TIMER1_CTL_R = 0x00000001;    // 10) enable TIMER1A
}

// Disable Systick timer
void Timer1A_Stop(void){
    TIMER1_CTL_R = 0x00000000;
}


