/*******************************************************************************
Confidential Proprietary information
Copyright 2002-2005

Project : STK500DaugherCard    
File    : MotorSystem.h
Author  : Erick L. Oberstar - Mechatronics Lab Manager / Electrical Engineer
Company : University of Wisconsin - Madison
		1513 Univ. Ave. Madison WI 53706 
********************************************************************************

		           
Revision history    Date         Name        		Reason
        -------    --------     ------      		-----------------------
          1.00     02/05/02     Erick L. Oberstar	Development Release
          1.10     11/04/05     Erick L. Oberstar	ME601 Course 
							Modifications

This file contains definitions for access of various Motor Systems Status and 
Command Signals via the STK500-DaughterBoard

The motor system command/control & feedback signals are wired over the molex 
screw connector from a motor system into the STK500 Development Boxes.

Typically the motor system amplifier control signals, indicator lights, and 
other outputs to the motor system are connected to the STK500 Daughter Card 
LTCHOUT (WRLATCH) digital output port via a 10-pin MTA type connector.  

The motor system switches and status signals are typically wired into the 
RDSTATUS digital input port on the STK500 Daughter Card  LTCHOUT port
via a 10-pin MTA type connector.  


The Encoder feedback signals are hard wired into the encoder channel 0 on the
STK500 Daughter Card.

The analog output DAC channel 0 is hard wired up to 
the motor command voltage on the molex motor system interface connector.


*******************************************************************************/


#ifndef motorsys_included
#define motorsys_included	// Sets that MotorSystem.hhas been included



// Define Bit Mapping to Status Byte and Command Byte
// Status Byte
#define Red_Button 0x80		// Bit 7 Red Button Mask
#define Black_Button 0x40	// Bit 6 Black Button Mask
#define Black_Button1 0x40      // Bit 6 Black Button #1 Mask on Inertia Disk Only Motor Sys
#define Soft_Lim_Left 0x20	// Bit 5 Left Soft Limit Switch Mask 
#define Black_Button2 0x20      // Bit 5 Black Button #2 Mask on Inertia Disk Only Motor Sys
#define Soft_Lim_Right 0x10	// Bit 4 Right Soft Limit Switch Mask
#define ToggleSw 0x10           // Bit 4 Toggle Switch on Inertia Disk Only Motor Sys
#define Pencil_Load 0x08	// Bit 3 Pencil Load Limit Switch Mask
#define Hard_Lim_Left 0x04	// Bit 2 Left Hard Limit Switch Mask
#define Hard_Lim_Right 0x02	// Bit 1 Right Hard Limit Switch Mask
#define Motor_Fault 0x01	// Bit 0 Motor Fault Signal Mask
// Command Byte
#define Sharpener 0b11110111	// Bit 3 to turn on the sharpener  (Active Low)
#define Inhibit 0b11111011 	// Bit 2 to actively inhibit  (Active Low)
#define YellowLED 0b11111101	// Bit 1 turn yellow light on (Active Low)
#define GreenLED 0b11111110 	// Bit 0 turn green light on  (Active Low)

//		WRPencilCommand(0b11111111);	// Deactivate all outputs &Enable motor (logic inverted in hardware)

unsigned char RDMotorSysStatus(void);	// Return byte value of Motor System
								//status byte

void WRMotorSysCommand(unsigned char);	// Writes a byte to the latched output port(Motor System Command pins)
								// on the STK500 Daughter Card.

#endif	//motorsys_included	