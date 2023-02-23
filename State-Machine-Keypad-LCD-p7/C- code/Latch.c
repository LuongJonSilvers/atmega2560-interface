// Function to drive WRLATCH 
#include <mega2560.h>
#include <stdio.h>
#include <delay.h>
#include "Latch.h"
void writeLatch(char data){
PORTK = 0x36;   //Place an address for the desired output bank on the decoder inputs – with the gate signal HIGH (Disabling the decoder chip)
DDRL=(1<<DDL7) | (1<<DDL6) | (1<<DDL5) | (1<<DDL4) | (1<<DDL3) | (1<<DDL2) | (1<<DDL1) | (1<<DDL0); //Configure the data bus PORT as an output
PORTL = data;  //Place the output value on the data bus PORT
delay_us(1);     //Wait for the data bus to stabilize
PORTK = 0x16;//Clear the decoder gate signal 
delay_us(1);// Wait for the data bus to propagate to the external side 
PORTK =  0x36;  // Set the gate signal to disable the decoder chip
}
