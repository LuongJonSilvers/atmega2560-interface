/*******************************************************
This program was created by the CodeWizardAVR V3.38 UL 
Automatic Program Generator
© Copyright 1998-2019 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : Serial Port-PC Communication
Version : 1.0
Date    : 11/16/2022
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


#include <mega2560.h>


#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE (1<<RXC0)
#define FRAMING_ERROR (1<<FE0)
#define PARITY_ERROR (1<<UPE0)
#define DATA_OVERRUN (1<<DOR0)

// USART0 Receiver buffer
#define RX_BUFFER_SIZE0 8


// USART0 Transmitter buffer
#define TX_BUFFER_SIZE0 16



char getchar(void);
void putchar(char c);
void usart0_init(void);
