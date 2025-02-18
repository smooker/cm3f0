#include "led.h"
#include <libopencm3/stm32/usart.h>

extern uint8_t aTxBuffer[];

//
void allsegmentsoff()
{
    for(int i=0;i<8;i++) {
        aTxBuffer[i] = __null;
    }
}

//decimal to symbol table
uint8_t d2s(uint8_t input)
{
    if (input == 0) {
        return __0;
    }
    else if (input == 1) {
        return __1;
    }
    else if (input == 2) {
        return __2;
    }
    else if (input == 3) {
        return __3;
    }
    else if (input == 4) {
        return __4;
    }
    else if (input == 5) {
        return __5;
    }
    else if (input == 6) {
        return __6;
    }
    else if (input == 7) {
        return __7;
    }
    else if (input == 8) {
        return __8;
    }
    else if (input == 9) {
        return __9;
    }
    else if (input == 10) {
        return __A;
    }
    else if (input == 11) {
        return __b;
    }
    else if (input == 12) {
        return __C;
    }
    else if (input == 13) {
        return __d;
    }
    else if (input == 14) {
        return __E;
    }
    else if (input == 15) {
        return __F;
    }

    else {
        return __o;
    }
}

//uint32_t to LED
void hex2led(uint32_t input)
{
    uint32_t start = 0x10000000;
    uint8_t digit = 0x00;
    uint8_t pos = 7;

    // 7 digits?
    while( start > 0 )
    {
        digit = input / start;
        aTxBuffer[pos--] = d2s(digit);
        input -= digit * start;
        start /= 0x10;
    }
}

//dot turned on on index ?
uint8_t getDot(uint8_t ind)
{
    if ( (aTxBuffer[ind] & 0x80) == 0x80) {         //8 bit
        return 1;         //true
    } else {
        return 0;       //false = 0
    }
}

//turn on dot on
void dot(uint8_t ind, uint8_t light)
{
    if (light > 0) {
        aTxBuffer[ind] |= 0x80;     //turn ON the dot
    } else {
        aTxBuffer[ind] &= 0x7f;     //turn OFF the dot
    }
}

//decimal to LED
void dec2led(int input)
{
    uint32_t start = 1000000;
    uint8_t digit = 0x00;
    uint8_t pos = 6;

    // 7 digits
    if (input < 0) {
        input *= -1;
        start = 100000;
        aTxBuffer[pos--] = __minus;
    }
    while( start > 0 )
    {
        digit = input / start;
        aTxBuffer[pos--] = d2s(digit);
        input -= digit * start;
        start /= 10;
    }
}
