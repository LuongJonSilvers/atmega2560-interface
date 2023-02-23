/*******************************************************
This program was created by the CodeWizardAVR V3.38 UL 
Automatic Program Generator
� Copyright 1998-2019 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : Fixed Point Closed Loop Motion Control System
Version : 1.0 
Date    : 12/07/2022
Author  : Viral Yemul & Jonathan Luong
Company : Mechatronics In Control & Product Realization
Comments: 


Chip type               : ATmega2560
Program type            : Application
AVR Core Clock frequency: 16.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 2048
*******************************************************/

#asm
.equ __lcd_port=0x08
#endasm
#include <lcd4x40.h>
#include <mega2560.h>
#include <string.h>
#include "USART.h"
#include <delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <motorsystem.h>
#include <math.h>

// Declare your global variables here

uint8_t echo_Character = 0x00;
uint16_t number_of_samples = 0;
uint16_t sample_holder = 0;
uint8_t MSByte = 0;
uint8_t LSByte = 0;
char eight = 0x00;
int32_t thirty_two = 0x00000000;
unsigned char lcd_string[32];
#if RX_BUFFER_SIZE0 < 256
extern volatile unsigned char rx_counter0;
#else
extern volatile unsigned int rx_counter0;
#endif


//declare global motor stuff
char valueSaw=0;
uint16_t encoder_count = 0;
uint16_t theta_past =0;
uint16_t theta_present=0;
int delta;
int actual_position;
int position;
float rpm;
char str[10];
bool buttonPressed;
char valueToOutput =128;
//declare close loop feedback stuff
signed int FB_reference_value=0;
signed int FB_error;
float Kp = 0.02187;
float Kd = 0.00009 ;
float K0, K1;
float Tsample = .00333;
signed int FB_error_past = 0;
//float FB_M;
//(IPF D on PV)
float K2;
int past_position = 0;
//extra credit
float KI=0.0143433;
float K3=0;
float K4=0;
signed int FB_M_past=0;
int past_past_position =0;

//Lab 13 
signed int K0_fixed;
signed int K1_fixed;
long  FB_M,val_0,val_1;







// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))
void writeLatch(char data){
    PORTK = 0x36;   //Place an address for the desired output bank on the decoder inputs � with the gate signal HIGH (Disabling the decoder chip)
    DDRL=(1<<DDL7) | (1<<DDL6) | (1<<DDL5) | (1<<DDL4) | (1<<DDL3) | (1<<DDL2) | (1<<DDL1) | (1<<DDL0); //Configure the data bus PORT as an output
    PORTL = data;  //Place the output value on the data bus PORT
    delay_us(1);     //Wait for the data bus to stabilize
    PORTK = 0x16;//Clear the decoder gate signal
    delay_us(1);// Wait for the data bus to propagate to the external side
    PORTK =  0x36;  // Set the gate signal to disable the decoder chip
}

void reset_encoder() // code for resetting the encoder value
{
    PORTK=0x24;
    delay_us(2);
    PORTK=0x04;
    delay_us(2);
    
    PORTK=0x25;
    delay_us(2);
    PORTK=0x05;
    delay_us(2);

}
// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
    ADMUX=(adc_input & 0x1f) | ADC_VREF_TYPE;
    if (adc_input & 0x20) ADCSRB|=(1<<MUX5);
    else ADCSRB&= ~(1<<MUX5);
    // Delay needed for the stabilization of the ADC input voltage
    delay_us(10);
    // Start the AD conversion
    ADCSRA|=(1<<ADSC);
    // Wait for the AD conversion to complete
    while ((ADCSRA & (1<<ADIF))==0);
    ADCSRA|=(1<<ADIF);
    return ADCW;
}

void outputtoDA(char channel,char value){            //Q12
    char address,clearAddress;
    //get appropriate address for relative channel input
    switch(channel){
        case 0:
        address = 0x20;
        break;
        case 1:
        address = 0x21;
        break;
        case 2:
        address = 0x22;
        break;
        case 3:
        address = 0x23;
        break;
        default: //invalid channel
        return;
    }
    clearAddress= address-0x20;
    
    PORTK = address;     //Place an address for the desired D/A channel on the decoder inputs �with the gate signal high (disabling the decoder chip
    DDRL=(1<<DDL7) | (1<<DDL6) | (1<<DDL5) | (1<<DDL4) | (1<<DDL3) | (1<<DDL2) | (1<<DDL1) | (1<<DDL0);        //Configure the data bus PORT as an output
    PORTL = value;        //Place the output value on the data bus PORT
    delay_us(1);        //Wait for the data bus to stabilize(enables decoder)
    PORTK =  clearAddress;    //Clear the decoder gate signal
    delay_us(1);    //Wait for the decoder output to propagate to D/A chip
    PORTK = address;    //Set the gate signal to disable the decoder chip



}

uint16_t encoderOutput(char encoder_channel){
char encoder_address,encoder_clearAddress,DA_address,DA_clearAddress;
char value_from_encoder_H=0x00;
char value_from_encoder_L=0x00;
uint16_t temp_value_H = 0x0000;
uint16_t temp_value_L = 0x0000;
uint16_t full_value = 0x0000;

switch(encoder_channel){
case 0:
encoder_address = 0x26;
DA_address=0x20;
break;
case 1:
encoder_address = 0x28;
DA_address=0x21;
break;
case 2:
encoder_address = 0x2A;
DA_address=0x22;
break;
default: //invalid channel
return -1;
}

DA_clearAddress= DA_address-0x20; //gate signal low (enabling decoder chip)
encoder_clearAddress=encoder_address-0x20; //gate signal low (enabling decoder chip)

//encoder 
DDRL=(0<<DDL7) | (0<<DDL6) | (0<<DDL5) | (0<<DDL4) | (0<<DDL3) | (0<<DDL2) | (0<<DDL1) | (0<<DDL0);// configure data for the port L as input
//PORTL =0xFF;
PORTK=encoder_clearAddress; //  Place an address for the desired input bank on the decoder inputs �with the gate signal LOW (Enabling the decoder chip) PORTK=0x1C
//PORTL = 0xff;
delay_us(2); //Wait for buffer inputs to propagate from external side to the data bus and for the data bus to stabilize
value_from_encoder_H = PINL; //reading Data bus Pin PORTL = 0x3C
PORTK=encoder_address; //Set the gate signal to disable the decoder chip

//READ
value_from_encoder_H = value_from_encoder_H;
temp_value_H=(uint16_t)(value_from_encoder_H);
temp_value_H=temp_value_H<<8;

//READ UPPER  8 with upper 4 bits zero (4 bits of useful info)
    //full_value=full_value||(((uint16_t)value_from_encoder & 0b00001111)<<8);
    

//encoder read lower
DDRL=(0<<DDL7) | (0<<DDL6) | (0<<DDL5) | (0<<DDL4) | (0<<DDL3) | (0<<DDL2) | (0<<DDL1) | (0<<DDL0);// configure data for the port L as input
//PORTL =0xFF;
PORTK=encoder_clearAddress+0x01; //  Place an address for the desired input bank on the decoder inputs �with the gate signal LOW (Enabling the decoder chip) PORTK=0x1C
//PORTL = 0xff;
delay_us(2); //Wait for buffer inputs to propagate from external side to the data bus and for the data bus to stabilize
value_from_encoder_L = PINL; //reading Data bus Pin PORTL = 0x3C
PORTK=encoder_address+0x01; //Set the gate signal to disable the decoder chip
temp_value_L = (uint16_t)value_from_encoder_L;
full_value = temp_value_H | temp_value_L;


return full_value;


}

void read(void){
    if(rx_counter0>0){
        echo_Character=getchar();
        if(echo_Character==115){
            reset_encoder();
            FB_reference_value = 500;
            //lcd_clear();
            MSByte = getchar();
            LSByte = getchar();
            number_of_samples = MSByte;
            number_of_samples = number_of_samples<<8;
            number_of_samples = number_of_samples | LSByte;
            sample_holder = number_of_samples;
            
        }
        
    }
    
}

void sendpacket(char eight, int32_t thirty_two){
    
    char MSB1;
    char MSB2;
    char MSB3;
    char MSB4;
    
    MSB1=(thirty_two>>24);
    MSB2=(thirty_two>>16);
    MSB3=(thirty_two>>8);
    MSB4=(thirty_two);
    putchar(eight);
    putchar(MSB1);
    //delay_us(1000);
    putchar(MSB2);
    //delay_us(1000);
    putchar(MSB3);
    //delay_us(1000);
    putchar(MSB4);
}



// Timer1 output compare A interrupt service routine
interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{
// Place your code here


// Place your code here
//lab11
//sawtooth usart
    //if(sample_holder>0){
        //sendpacket(eight,thirty_two);
        //thirty_two= thirty_two+300;
        //eight = eight+10;
        //sample_holder--;
    //}

    //uint16_t tempDAValue = (encoderOutput(0))>>4;
    //char DAValue = tempDAValue;
    
    //PORTA=0xff;
    theta_present = (encoderOutput(0));
    delta= theta_present - theta_past;
    if(delta > 2048){
        delta = delta - 4096;
    }else if(delta<-2048){
        delta = delta + 4096;
    }
    actual_position = actual_position + delta;
    theta_past = theta_present;
    //PORTA=0x00;
    //rpm = ((((float)delta)*60*100)/2000);
    //ftoa(rpm,1,str);
    //lcd_clear();
    //lcd_gotoxy(0,0);
    //lcd_puts(str);        // Outout to display
    //

    
    //sendpacket(eight,thirty_two);
    //sample_holder--;
    //outputtoDA(0,encoder_count);
    //outputtoDA(0,DAValue);
    //outputtoDA(1,encoder_count);
    //outputtoDA(2,encoder_count);
    
    // Q14 Proportional Close loop//////////////
    //FB_error = FB_reference_value-actual_position  ;        //Error Calc 
    //FB_M = FB_error * Kp;            // manipulation calc
    //
    //if(FB_M >127){
    //FB_M = 127;
    //}
    //if(FB_M <-128){
        //FB_M= -128;
    //}
    //
    //if(sample_holder>0){
        //sendpacket(FB_M,actual_position);
        //sample_holder--;
    //}
    //
    //
    //
        //writeLatch(0xff);
    //outputtoDA(0, FB_M+128);
    ////////////////
    


        


    //read();
    //Q15-16//////////////////////////////////// PD on E 
    //FB_error = FB_reference_value-actual_position  ;        //Error Calc
    //FB_M = (FB_error * K0) + (K1*FB_error_past);// manipulation calc        
    //K0 = Kp + (Kd/Tsample);
    //K1 = -(Kd/Tsample);
    //
    //if(FB_M >127){
    //FB_M = 127;
    //}
    //if(FB_M <-128){
    //FB_M= -128;
    //}
    //FB_error_past= FB_error;
    //if(sample_holder>0){
        //sendpacket(FB_M,actual_position);
        //sample_holder--;
    //}
    //writeLatch(0xff);
    //outputtoDA(0, FB_M+128);
    ////////////////////////////////////////////////////
    
    //Q17-18-19/////////////////////////////////////// PD on PV
    
	//PORTA=0xff;
    //FB_error = FB_reference_value-actual_position  ;
	//K0=Kp;
    //K1 = -1*(Kd/Tsample);
    //K2=(Kd/Tsample);
    //FB_M = (FB_error * K0) + (K1*actual_position) + (K2*past_position);
    //past_position = actual_position;
    //if(FB_M >127){
    //FB_M = 127;
    //}
    //if(FB_M <-128){
    //FB_M= -128;
    //}
    //FB_error_past= FB_error;
    ////PORTA=0x00;
    ////PORTA=0xff;
    //if(sample_holder>0){
        //sendpacket(FB_M,actual_position);
        //sample_holder--;
    //}
    ////PORTA=0x00;
    //writeLatch(0xff);
    ////PORTA=0xff;
    //outputtoDA(0, FB_M+128);
   // PORTA=0x00;
    ///////////////////////////
    

    //Q20
    //FB_error = FB_reference_value-actual_position  ;
    //K0=Kp+(KI*Tsample);
    //K1=-1*Kp;
    //K2=-1*(Kd/Tsample);
    //K3=2*(Kd/Tsample);
    //K4=-1*(Kd/Tsample);
    //FB_M = FB_M_past+(FB_error * K0) + (K1*FB_error_past) + (K2*actual_position)+(K3*past_position)+(K4*past_past_position);
//FB_M_past=FB_M;
//FB_error_past=FB_error;    
////past_position=actual_position;
//past_past_position=past_position;
//past_position=actual_position;
//if(sample_holder>0){
        //sendpacket(FB_M,actual_position);
        //sample_holder--;
    //}
    //writeLatch(0xff);
    //outputtoDA(0, FB_M+128);
//
//lab 13 Q7
//convert floating point to fixed point
K0_fixed = K0*pow(2,12);
K1_fixed = K1*pow(2,12);

//lab 13 Q8
//PORTA = 0xff;
K0=Kp;
K1 = -1*(Kd/Tsample);
K2=(Kd/Tsample);
FB_error = FB_reference_value-actual_position;
val_0 = ((long)K0_fixed)* ((long)FB_error);
val_1 = ((long)K1_fixed)* ((long)delta);
FB_M = ((val_0>>4)+(val_1))>>16;

if(FB_M >127){
	FB_M = 127;
}
if(FB_M <-128){
	FB_M= -128;
}

//PORTA=0x00;
//PORTA=0xff;
if(sample_holder>0){
	sendpacket(FB_M,actual_position);
	sample_holder--;
}
//PORTA=0x00;
writeLatch(0xff);
PORTA=0xff;
outputtoDA(0, FB_M+128);
PORTA=0x00;

}



//void check_buttons(){
    //char encoder_address = 0x26;
    //char buttons = PINA;
    //if(PINA != 0xff){
        //PORTK=0b000100;
        //encoder_count = 0;
        //PORTK=encoder_address; //Set the gate signal to disable the decoder chip
        //delay_us(2);
    //}
    //
//}
//Digital I/O and DAC Code

//void outputtoDA(char channel,char value){
    //char address,clearAddress;
    ////get appropriate address for relative channel input
    //switch(channel){
        //case 0:
        //address = 0x20;
        //break;
        //case 1:
        //address = 0x21;
        //break;
        //case 2:
        //address = 0x22;
        //break;
        //case 3:
        //address = 0x23;
        //break;
        //default: //invalid channel
        //return;
    //}
    //clearAddress= address-0x20;
    //
    //PORTK = address;     //Place an address for the desired D/A channel on the decoder inputs �with the gate signal high (disabling the decoder chip
    //DDRL=(1<<DDL7) | (1<<DDL6) | (1<<DDL5) | (1<<DDL4) | (1<<DDL3) | (1<<DDL2) | (1<<DDL1) | (1<<DDL0);        //Configure the data bus PORT as an output
    //PORTL = value;        //Place the output value on the data bus PORT
    //delay_us(1);        //Wait for the data bus to stabilize(enables decoder)
    //PORTK =  clearAddress;    //Clear the decoder gate signal
    //delay_us(1);    //Wait for the decoder output to propagate to D/A chip
    //PORTK = address;    //Set the gate signal to disable the decoder chip
//
    //
//}


//void read(void){
    //if(rx_counter0>0){
        //echo_Character=getchar();
        //if(echo_Character==115){
            //lcd_clear();
            //MSByte = getchar();
            //LSByte = getchar();
            //number_of_samples = MSByte;
            //number_of_samples = number_of_samples<<8;
            //number_of_samples = number_of_samples | LSByte;
            //sample_holder = number_of_samples;
            //putchar(number_of_samples);
            //sprintf(lcd_string,"%u",number_of_samples);
            //lcd_puts(lcd_string);
        //}
        //
    //}
//}
//void echo(void){
    //Place your code here
    //if(rx_counter0>0){
        //echo_Character = getchar();
        //putchar(echo_Character);
    //}
//}

void main(void)
{
    char temp_PORTK = PINK;
    lcd_init();
    
// Declare your local variables here



// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

// Input/Output Ports initialization
// Port A initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRA=(1<<DDA7) | (1<<DDA6) | (1<<DDA5) | (1<<DDA4) | (1<<DDA3) | (1<<DDA2) | (1<<DDA1) | (1<<DDA0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRC=(1<<DDC7) | (1<<DDC6) | (1<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (1<<DDC0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Port E initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRE=(0<<DDE7) | (0<<DDE6) | (0<<DDE5) | (0<<DDE4) | (0<<DDE3) | (0<<DDE2) | (1<<DDE1) | (0<<DDE0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);

// Port F initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRF=(0<<DDF7) | (0<<DDF6) | (0<<DDF5) | (0<<DDF4) | (0<<DDF3) | (0<<DDF2) | (0<<DDF1) | (0<<DDF0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (0<<PORTF0);

// Port G initialization
// Function: Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRG=(0<<DDG5) | (0<<DDG4) | (0<<DDG3) | (0<<DDG2) | (0<<DDG1) | (0<<DDG0);
// State: Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTG=(0<<PORTG5) | (0<<PORTG4) | (0<<PORTG3) | (0<<PORTG2) | (0<<PORTG1) | (0<<PORTG0);

// Port H initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRH=(0<<DDH7) | (0<<DDH6) | (0<<DDH5) | (0<<DDH4) | (0<<DDH3) | (0<<DDH2) | (0<<DDH1) | (0<<DDH0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTH=(0<<PORTH7) | (0<<PORTH6) | (0<<PORTH5) | (0<<PORTH4) | (0<<PORTH3) | (0<<PORTH2) | (0<<PORTH1) | (0<<PORTH0);

// Port J initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRJ=(0<<DDJ7) | (0<<DDJ6) | (0<<DDJ5) | (0<<DDJ4) | (0<<DDJ3) | (0<<DDJ2) | (0<<DDJ1) | (0<<DDJ0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTJ=(0<<PORTJ7) | (0<<PORTJ6) | (0<<PORTJ5) | (0<<PORTJ4) | (0<<PORTJ3) | (0<<PORTJ2) | (0<<PORTJ1) | (0<<PORTJ0);

// Port K initialization
// Function: Bit7=In Bit6=In Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRK=(1<<DDK7) | (1<<DDK6) | (1<<DDK5) | (1<<DDK4) | (1<<DDK3) | (1<<DDK2) | (1<<DDK1) | (1<<DDK0);
// State: Bit7=T Bit6=T Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTK=(0<<PORTK7) | (0<<PORTK6) | (0<<PORTK5) | (0<<PORTK4) | (0<<PORTK3) | (0<<PORTK2) | (0<<PORTK1) | (0<<PORTK0);

// Port L initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRL=(1<<DDL7) | (1<<DDL6) | (1<<DDL5) | (1<<DDL4) | (1<<DDL3) | (1<<DDL2) | (1<<DDL1) | (1<<DDL0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTL=(0<<PORTL7) | (0<<PORTL6) | (0<<PORTL5) | (0<<PORTL4) | (0<<PORTL3) | (0<<PORTL2) | (0<<PORTL1) | (0<<PORTL0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=0xFF
// OC0A output: Disconnected
// OC0B output: Disconnected
TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;
OCR0A=0x00;
OCR0B=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 250.000 kHz
// Mode: CTC top=OCR1A
// OC1A output: Disconnected
// OC1B output: Disconnected
// OC1C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 3.336 ms
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: On
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x03;
OCR1AL=0x41;
OCR1BH=0x00;
OCR1BL=0x00;
OCR1CH=0x00;
OCR1CL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer2 Stopped
// Mode: Normal top=0xFF
// OC2A output: Disconnected
// OC2B output: Disconnected
ASSR=(0<<EXCLK) | (0<<AS2);
TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2A=0x00;
OCR2B=0x00;

// Timer/Counter 3 initialization
// Clock source: System Clock
// Clock value: Timer3 Stopped
// Mode: Normal top=0xFFFF
// OC3A output: Disconnected
// OC3B output: Disconnected
// OC3C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer3 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR3A=(0<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (0<<WGM31) | (0<<WGM30);
TCCR3B=(0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (0<<WGM32) | (0<<CS32) | (0<<CS31) | (0<<CS30);
TCNT3H=0x00;
TCNT3L=0x00;
ICR3H=0x00;
ICR3L=0x00;
OCR3AH=0x00;
OCR3AL=0x00;
OCR3BH=0x00;
OCR3BL=0x00;
OCR3CH=0x00;
OCR3CL=0x00;

// Timer/Counter 4 initialization
// Clock source: System Clock
// Clock value: Timer4 Stopped
// Mode: Normal top=0xFFFF
// OC4A output: Disconnected
// OC4B output: Disconnected
// OC4C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer4 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR4A=(0<<COM4A1) | (0<<COM4A0) | (0<<COM4B1) | (0<<COM4B0) | (0<<COM4C1) | (0<<COM4C0) | (0<<WGM41) | (0<<WGM40);
TCCR4B=(0<<ICNC4) | (0<<ICES4) | (0<<WGM43) | (0<<WGM42) | (0<<CS42) | (0<<CS41) | (0<<CS40);
TCNT4H=0x00;
TCNT4L=0x00;
ICR4H=0x00;
ICR4L=0x00;
OCR4AH=0x00;
OCR4AL=0x00;
OCR4BH=0x00;
OCR4BL=0x00;
OCR4CH=0x00;
OCR4CL=0x00;

// Timer/Counter 5 initialization
// Clock source: System Clock
// Clock value: Timer5 Stopped
// Mode: Normal top=0xFFFF
// OC5A output: Disconnected
// OC5B output: Disconnected
// OC5C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer5 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR5A=(0<<COM5A1) | (0<<COM5A0) | (0<<COM5B1) | (0<<COM5B0) | (0<<COM5C1) | (0<<COM5C0) | (0<<WGM51) | (0<<WGM50);
TCCR5B=(0<<ICNC5) | (0<<ICES5) | (0<<WGM53) | (0<<WGM52) | (0<<CS52) | (0<<CS51) | (0<<CS50);
TCNT5H=0x00;
TCNT5L=0x00;
ICR5H=0x00;
ICR5L=0x00;
OCR5AH=0x00;
OCR5AL=0x00;
OCR5BH=0x00;
OCR5BL=0x00;
OCR5CH=0x00;
OCR5CL=0x00;

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (0<<TOIE0);

// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=(0<<ICIE1) | (0<<OCIE1C) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);

// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);

// Timer/Counter 3 Interrupt(s) initialization
TIMSK3=(0<<ICIE3) | (0<<OCIE3C) | (0<<OCIE3B) | (0<<OCIE3A) | (0<<TOIE3);

// Timer/Counter 4 Interrupt(s) initialization
TIMSK4=(0<<ICIE4) | (0<<OCIE4C) | (0<<OCIE4B) | (0<<OCIE4A) | (0<<TOIE4);

// Timer/Counter 5 Interrupt(s) initialization
TIMSK5=(0<<ICIE5) | (0<<OCIE5C) | (0<<OCIE5B) | (0<<OCIE5A) | (0<<TOIE5);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: Off
// INT3: Off
// INT4: Off
// INT5: Off
// INT6: Off
// INT7: Off
EICRA=(0<<ISC31) | (0<<ISC30) | (0<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (0<<ISC51) | (0<<ISC50) | (0<<ISC41) | (0<<ISC40);
EIMSK=(0<<INT7) | (0<<INT6) | (0<<INT5) | (0<<INT4) | (0<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);
// PCINT0 interrupt: Off
// PCINT1 interrupt: Off
// PCINT2 interrupt: Off
// PCINT3 interrupt: Off
// PCINT4 interrupt: Off
// PCINT5 interrupt: Off
// PCINT6 interrupt: Off
// PCINT7 interrupt: Off
// PCINT8 interrupt: Off
// PCINT9 interrupt: Off
// PCINT10 interrupt: Off
// PCINT11 interrupt: Off
// PCINT12 interrupt: Off
// PCINT13 interrupt: Off
// PCINT14 interrupt: Off
// PCINT15 interrupt: Off
// PCINT16 interrupt: Off
// PCINT17 interrupt: Off
// PCINT18 interrupt: Off
// PCINT19 interrupt: Off
// PCINT20 interrupt: Off
// PCINT21 interrupt: Off
// PCINT22 interrupt: Off
// PCINT23 interrupt: Off
PCMSK0=(0<<PCINT7) | (0<<PCINT6) | (0<<PCINT5) | (0<<PCINT4) | (0<<PCINT3) | (0<<PCINT2) | (0<<PCINT1) | (0<<PCINT0);
PCMSK1=(0<<PCINT15) | (0<<PCINT14) | (0<<PCINT13) | (0<<PCINT12) | (0<<PCINT11) | (0<<PCINT10) | (0<<PCINT9) | (0<<PCINT8);
PCMSK2=(0<<PCINT23) | (0<<PCINT22) | (0<<PCINT21) | (0<<PCINT20) | (0<<PCINT19) | (0<<PCINT18) | (0<<PCINT17) | (0<<PCINT16);
PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);


// USART0 initialization
// USART0 disabled
//UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);

// USART1 initialization
// USART1 disabled
UCSR1B=(0<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (0<<RXEN1) | (0<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);

// USART2 initialization
// USART2 disabled
UCSR2B=(0<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (0<<RXEN2) | (0<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82);

// USART3 initialization
// USART3 disabled
UCSR3B=(0<<RXCIE3) | (0<<TXCIE3) | (0<<UDRIE3) | (0<<RXEN3) | (0<<TXEN3) | (0<<UCSZ32) | (0<<RXB83) | (0<<TXB83);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
ADCSRB=(0<<ACME);
// Digital input buffer on AIN0: On
// Digital input buffer on AIN1: On
DIDR1=(0<<AIN0D) | (0<<AIN1D);

// ADC initialization
// ADC disabled
ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

usart0_init();

// Globally enable interrupts
#asm("sei")

PORTK= temp_PORTK | 0b000100;
delay_us(2);
reset_encoder();

//lab 13 Q7
//convert floating point to fixed point
K0_fixed = K0*pow(2,12);
K1_fixed = K1*pow(2,12);

while (1)
      {
       
  
      // Place your code here
     //sendpacket(eight,thirty_two);

        //echo();
    read();
    
        //check_buttons();
        //writeLatch(0xff);
        //outputtoDA(0,255);
      }
}
