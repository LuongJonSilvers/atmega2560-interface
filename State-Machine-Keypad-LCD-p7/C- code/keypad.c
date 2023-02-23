/**
#include "keypad.h"
unsigned int checkkeypad(void){
char data;
int button;
writeLatch(0b11101111);
data = readStatus();
button = ((unsigned int)(data & 0xF0))>>4;
writeLatch(0b11011111);
data = readStatus();
button |= ((unsigned int)(data & 0xF0));
writeLatch(0b10111111);
data = readStatus();
button |= ((unsigned int)(data & 0xF0))<<4;
writeLatch(0b01111111);
data = readStatus();
button |= ((unsigned int)(data & 0xF0))>>8;
return button;
}
**/