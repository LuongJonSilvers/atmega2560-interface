/*******************************************************
This program was created by the CodeWizardAVR V3.38 UL
Automatic Program Generator
� Copyright 1998-2019 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : Encoder Counter Driver
Version : 1.0
Date    : 11/9/2022
Author  : Jonathan Luong & Viral Yemul
Company : Mechatronics In Control & Product Realization
Comments:


Chip type               : ATmega2560
Program type            : Application
AVR Core Clock frequency: 16.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 2048
*******************************************************/

// Function to drive WRLATCH 
#include <mega2560.h>
#include <stdio.h>
#include <delay.h>
#include "Latch.h"
void writeLatch(char data){
PORTK = 0x36;   //Place an address for the desired output bank on the decoder inputs � with the gate signal HIGH (Disabling the decoder chip)
DDRL=(1<<DDL7) | (1<<DDL6) | (1<<DDL5) | (1<<DDL4) | (1<<DDL3) | (1<<DDL2) | (1<<DDL1) | (1<<DDL0); //Configure the data bus PORT as an output
PORTL = data;  //Place the output value on the data bus PORT
delay_us(1);     //Wait for the data bus to stabilize
PORTK = 0x16;//Clear the decoder gate signal 
delay_us(1);// Wait for the data bus to propagate to the external side 
PORTK =  0x36;  // Set the gate signal to disable the decoder chip
}
