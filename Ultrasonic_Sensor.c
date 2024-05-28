// Ultrasonic_Sensor.c
// This program runs on TM4C123.
// This is an example program to show how to interface HC-SR04 Ultrasonic sensor.
// PB6 connects to echo pin to generate edge-triggered interrupt.
// PB7 connects to Ultrasonic sensor trigger pin.
// SysTick timer is used to generate the required timing for trigger pin and measure echo pulse width.
// Global variable "distance" is used to hold the distance in cemtimeter for the obstacles
// in front of the sensor. 
// By Dr. Min He
// December 10th, 2018

#include <stdbool.h>
#include <stdint.h>
#include "SysTick.h"
#include "Timer1A.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"

#define TRIGGER_PIN 		(*((volatile unsigned long *)0x40005080))  // PB7 is the trigger pin	
#define TRIGGER_VALUE 	0x20   // trigger at bit 7
#define ECHO_PIN 				(*((volatile unsigned long *)0x40005040))  // PB6 is the echo pin	
#define ECHO_VALUE 			0x10   // trigger at bit 6
#define MC_LEN 					0.0625 // length of one machine cycle in microsecond for 16MHz clock
#define SOUND_SPEED 		0.0343 // centimeter per micro-second
#define LED       (*((volatile unsigned long *)0x40025038)) // bit address for RBG LEDs
#define RED 				0x02
#define BLUE 				0x04
#define GREEN 			0x08
#define OFF  				0x00
#define WHITE       0x0E
#define PERIOD      80000
#define OUTOFRANGE 90
bool flag = false;
bool stopped = false;
bool timing = false;


extern void SysTick_Wait_Quarter(uint8_t);
extern void EnableInterrupts(void);
extern void GPIOPortB_Handler(void);
extern void Quarter_Sec();
void PortB_Init(void);
void PortF_Init(void);
void SysTick_Handler(void);
void Quarter_Sec();

static volatile uint8_t done=0;
static volatile uint32_t distance=0;
int LIGHT = 0;
unsigned long int L;
unsigned long int H;
int SECONDSTAGE = 0;

int main(void){
	PortF_Init();
	PortB_Init();
  EnableInterrupts();
	Timer1A_Init();
	PLL_Init();
	SysTick_Start();
  while(1){
		done = 0;
		distance = 0;
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
		//SysTick_Wait1us(2);
		GPTM_Wait1us(2);
		TRIGGER_PIN |= TRIGGER_VALUE; // send high to trigger
		//SysTick_Wait1us(10);
		GPTM_Wait1us(10);
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
		while (!done);
		
		if (distance < 10){
			SECONDSTAGE = 2;
			LED = RED;
			Quarter_Sec();
			LED = OFF;
			Quarter_Sec();
			//SysTick_Wait_Quarter(1);
			
			
		}
		else if (distance > 70){
			LIGHT = OFF;
			LED = OFF;
			SECONDSTAGE = 0;
		}
		else{
			LIGHT = WHITE;
			SECONDSTAGE = 1;
		}
		
		if((distance >= 10) &(distance < 20)){
			H = PERIOD/4 * 1;
		}
		else if(distance < 30){
			H = PERIOD/4 * 0.8;
		}
		else if(distance < 40){
			H = PERIOD/4 * 0.6;
		}
		else if(distance < 50){
			H = PERIOD/4 * 0.4;
		}
		else if(distance < 60){
			H = PERIOD/4 * 0.2;
		}
		else if(distance <= 70){
			H = PERIOD/4 * 0.1;
		}
		L = PERIOD - H;
    
  }
}








void PortB_Init(void){ 
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;           // 1) activate clock for Port A
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)!=SYSCTL_RCGC2_GPIOB){}; // wait for clock to start
                                    // 2) no need to unlock PA2
  GPIO_PORTB_PCTL_R &= ~0x00FF0000; // 3) regular GPIO
  GPIO_PORTB_AMSEL_R &= (uint32_t)~0x30;      // 4) disable analog function on PA2
  GPIO_PORTB_DIR_R &= ~0x10;        // 5) PB6:echo pin, input
  GPIO_PORTB_DIR_R |= 0x20;         // 5) PB7:trigger pin, output
  GPIO_PORTB_AFSEL_R &= ~0x30;      // 6) regular port function
  GPIO_PORTB_DEN_R |= 0x30;         // 7) enable digital port
  GPIO_PORTB_IS_R &= ~0x10;         // PB6 is edge-sensitive
  GPIO_PORTB_IBE_R |= 0x10;         // PB6 is both edges
  GPIO_PORTB_IEV_R &= ~0x10;        // PB6 both edge event
  GPIO_PORTB_ICR_R = 0x10;          // clear flag 6
  GPIO_PORTB_IM_R |= 0x10;          // arm interrupt on PB6
  NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFF1FFF)|0x00008000; // (g) priority 3
  NVIC_EN0_R = 0x00000002;      // (h) enable Port B edge interrupt
}

void PortF_Init(void){
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;  // 1) activate clock for Port F
	while((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)!=SYSCTL_RCGC2_GPIOF){}; // wait for clock to start

	GPIO_PORTF_PCTL_R &= ~0x0FFF0;
	GPIO_PORTF_AMSEL_R &= (uint32_t)~0x0E;      // 4) disable analog function on P123
  GPIO_PORTF_DIR_R |= 0x0E;         // 5) PB123:LEDS, output
  GPIO_PORTF_AFSEL_R &= ~0x0E;      // 6) regular port function
  GPIO_PORTF_DEN_R |= 0x0E;         // 7) enable digital port
}

void GPIOPortB_Handler(void){
	//timing = true;
	flag = false;
	TIMER1_IMR_R = 0x01;
	if (ECHO_PIN==ECHO_VALUE){  // echo pin rising edge is detected, start timing
		//SysTick_Start();
		Timer1A_Start();
		stopped = false;
	}
	else { 
		Timer1A_Stop();
		stopped = true;
		//distance = (uint32_t)(SysTick_Get_MC_Elapsed()*MC_LEN*SOUND_SPEED)/2;
		if(!flag){
			distance = (uint32_t)(GPTM_Get_MC_Elapsed()*MC_LEN*SOUND_SPEED)/2;		
			//flag = false;
			TIMER1_IMR_R = 0x00;
		}else {
			distance = OUTOFRANGE;
		}
		done = 1;
	}
	TIMER1_IMR_R = 0x00;
  GPIO_PORTB_ICR_R = 0x10;      // acknowledge flag 6

}

void Timer1A_Handler(void){
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER1A timeout
	flag = true;
}


void SysTick_Handler(void){
	if(SECONDSTAGE == 1){
		NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;// turn off SysTick 
		if(LED == WHITE){   // previous cycle is duty cycle
			LED = OFF;
			NVIC_ST_RELOAD_R = L-1;     // switch to non-duty cycle
		} else { // previous cycle is not duty cycle
			LED = WHITE;
			NVIC_ST_RELOAD_R = H-1;     // switch to duty cycle
		}
		NVIC_ST_CURRENT_R = 0;
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // turn on systick to continue
	}

}
