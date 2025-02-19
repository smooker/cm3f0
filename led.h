#ifndef LED_H
#define LED_H

#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#include <sys/types.h>

#include "ledsegments.h"

#define BKPT asm("bkpt 255")

void allsegmentsoff(void);
uint8_t d2s(uint8_t input);
void hex2led(uint32_t input);
uint8_t getDot(uint8_t ind);
void dot(uint8_t ind, uint8_t light);
void dec2led(int input);
void float2led(float input);
void brightness (uint8_t input);

#endif // LED_H
