#include "led.h"
#include <libopencm3/stm32/usart.h>

extern uint8_t aTxBuffer[];
extern uint8_t aTxBufferPos;

/**
  * @brief
  * @retval
  */
void allsegmentsoff()
{
    for(int i=0;i<8;i++) {
        aTxBuffer[i] = __null;
    }
}

/**
  * @brief decimal to symbol function xref table
  * @retval
  */
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

/**
  * @brief uint32_t to LED
  * @retval
  */
void hex2led(uint32_t input)
{
    uint32_t start = 0x10000000;
    uint8_t digit = 0x00;
    uint8_t pos = 7;

    aTxBufferPos = 1;

    // 7 digits?
    while( start > 0 )
    {
        digit = input / start;
        aTxBuffer[pos--] = d2s(digit);
        input -= digit * start;
        start /= 0x10;
    }
    aTxBufferPos = 0;
}

/**
  * @brief dot turned on on index
  * @retval
  */
uint8_t getDot(uint8_t ind)
{
    if ( (aTxBuffer[ind] & 0x80) == 0x80) {         //8 bit
        return 1;         //true
    } else {
        return 0;       //false = 0
    }
}

/**
  * @brief turn on dot on
  * @retval
  */
void dot(uint8_t ind, uint8_t light)
{
    if (light > 0) {
        aTxBuffer[ind] |= 0x80;     //turn ON the dot
    } else {
        aTxBuffer[ind] &= 0x7f;     //turn OFF the dot
    }
}

/**
  * @brief float to LED
  * @retval
  */
void float2led(float input)
{
    int resultS = input;
    int result = resultS;
    int8_t sign = 1;


    if ( result < 0 ) {
        result *= -1;
        sign = -1;
        input *= -1.0f;
        resultS = result;
    }

    //check for sign

    if ( result == 0 ) {
        result = (int) (input * 100000);
    }
    else if ( (result >= 1) & (result <= 9) ) {
        result = (int) (input * 100000);
    }
    else if ( (result >= 10) & (result <= 99) ) {
        result = (int) (input * 10000);
    }
    else if ( (result >= 100) & (result <= 999) ) {
        result = (int) (input * 1000);
    }
    else if ( (result >= 1000) & (result <= 9999) ) {
        result = (int) (input * 100);
    }
    else if ( (result >= 10000) & (result <= 99999) ) {
        result = (int) (input * 10);
    }
    else if ( (result >= 100000) & (result <= 999999) ) {
        result = (int) (input * 1);
    }

    result *= sign;

    dec2led(result);

    result *= sign;

    //putdot
    if ( resultS == 0 ) {
        dot(5, 1);
    }
    else if ( (resultS >= 1) & (resultS <= 9) ) {
        dot(5, 1);
    }
    else if ( (resultS >= 10) & (resultS <= 99) ) {
        dot(4, 1);
    }
    else if ( (resultS >= 100) & (resultS <= 999) ) {
        dot(3, 1);
    }
    else if ( (resultS >= 1000) & (resultS <= 9999) ) {
        dot(2, 1);
    }
    else if ( (resultS >= 10000) & (resultS <= 99999) ) {
        dot(1, 1);
    }
    else if ( (resultS >= 100000) & (resultS <= 999999) ) {
        dot(0, 1);
    }
}

/**
  * @briefdecimal to LED
  * @retval
  */
void dec2led(int input)
{
    uint32_t start = 1000000;
    uint8_t digit = 0x00;
    uint8_t pos = 6;

    aTxBufferPos = 1;

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
    aTxBufferPos = 0;
}
