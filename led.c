//
void allsegmentsoff()
{
    for(int i=0;i<8;i++) {
        aTxBuffer[i] = _null;
    }
}

//decimal to symbol table
uint8_t d2s(uint8_t input)
{
    if (input == 0) {
        return _0;
    }
    else if (input == 1) {
        return _1;
    }
    else if (input == 2) {
        return _2;
    }
    else if (input == 3) {
        return _3;
    }
    else if (input == 4) {
        return _4;
    }
    else if (input == 5) {
        return _5;
    }
    else if (input == 6) {
        return _6;
    }
    else if (input == 7) {
        return _7;
    }
    else if (input == 8) {
        return _8;
    }
    else if (input == 9) {
        return _9;
    }
    else if (input == 10) {
        return _A;
    }
    else if (input == 11) {
        return _b;
    }
    else if (input == 12) {
        return _C;
    }
    else if (input == 13) {
        return _d;
    }
    else if (input == 14) {
        return _E;
    }
    else if (input == 15) {
        return _F;
    }

    else {
        return _o;
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
        return SET;         //true
    } else {
        return RESET;       //false = 0
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
        aTxBuffer[pos--] = _minus;
    }
    while( start > 0 )
    {
        digit = input / start;
        aTxBuffer[pos--] = d2s(digit);
        input -= digit * start;
        start /= 10;
    }
}
