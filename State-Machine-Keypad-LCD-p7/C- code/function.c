#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>

#define testPattern 0b011011000110111

int positionOfFirstZero;
int shifts;
int mask;

void WRLATCH_function(uint8_t value){
    
}

int main(){
for(int i = 0; i<16;i++){
    mask = (1 << i);
    if(!(testPattern&mask)){
    positionOfFirstZero = i;
    break;
    }else{
        shifts++;
    }

}
switch(positionOfFirstZero)
    {
        case 0:
            printf("1");
            break;
        case 1:
            printf("4");
            break;
        case 2:
            printf("7");
            break;
        case 3:
            printf("*");
            break;
        case 4:
            printf("2");
            break;
        case 5:
            printf("5");
            break;
        case 6:
            printf("8");
            break;
        case 7:
            printf("0");
            break;
        case 8:
            printf("3");
            break;
        case 9:
            printf("6");
            break;
        case 10:
            printf("9");
            break;
        case 11:
            printf("#");
            break;
        case 12:
            printf("A");
            break;
        case 13:
            printf("B");
            break;
        case 14:
            printf("C");
            break;
        case 15:
            printf("D");
            break;
            
    }

}
