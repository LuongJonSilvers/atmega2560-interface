#include <mega2560.h>
#include <stdio.h>
#include <delay.h>
#define readStatusHigh 0x1C
#define readStatusLow 0x3C
#define addressBus PORTK
#define dataBus PORTL
#include "RdStatus.h"
unsigned char readStatus (void){
	unsigned char data;
	DDRL=(0<<DDL7) | (0<<DDL6) | (0<<DDL5) | (0<<DDL4) | (0<<DDL3) | (0<<DDL2) | (0<<DDL1) | (0<<DDL0);// configure data for the port L as input 
	addressBus = readStatusHigh ; //  Place an address for the desired input bank on the decoder inputs –with the gate signal LOW (Enabling the decoder chip)
	//PORTL = 0xff;
	delay_us(1); //Wait for buffer inputs to propagate from external side to the data bus and for the data bus to stabilize
	data = PINL & 0xf0; //reading Data bus Pin 
	addressBus = readStatusLow; //Set the gate signal to disable the decoder chip
	return data;
	
}