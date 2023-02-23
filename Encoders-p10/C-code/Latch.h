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

#include <mega2560.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <stringconversion.h>
#include <lcd4x40.h>
#include <delay.h>
#include <stdint.h>


void writeLatch(uint8_t data);