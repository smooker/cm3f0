/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Modified by Fernando Cortes <fermando.corcam@gmail.com>
 * modified by Guillermo Rivera <memogrg@gmail.com>
 * modified by Frantisek Burian <BuFran@seznam.cz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

//MYMACROS #############################
/* Some test definition here */
#define DEFINED_BUT_NO_VALUE
#define DEFINED_INT 3
#define DEFINED_STR "ABC"

/* definition to expand macro then apply to pragma message */
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)

/* Some example here */
// #pragma message(VAR_NAME_VALUE(NOT_DEFINED))
// #pragma message(VAR_NAME_VALUE(DEFINED_BUT_NO_VALUE))
// #pragma message(VAR_NAME_VALUE(DEFINED_INT))
#pragma message(VAR_NAME_VALUE(__ARM_FP))

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>

#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#include <sys/types.h>

uint8_t channel_array[] = { 1, 1, ADC_CHANNEL_TEMP};


#pragma message(VAR_NAME_VALUE(NOT_DEFINED))

//##################
static ssize_t _iord(void *_cookie, char *_buf, size_t _n);
static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n);

static ssize_t _iord(void *_cookie, char *_buf, size_t _n)
{
    /* dont support reading now */
    (void)_cookie;
    (void)_buf;
    (void)_n;
    return 0;
}

static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n)
{
    uint32_t dev = (uint32_t)_cookie;

    int written = 0;
    while (_n-- > 0) {
        usart_send_blocking(dev, *_buf++);
        written++;
    };
    return written;
}

static FILE *usart_setup(uint32_t dev)
{
    /* Setup USART parameters. */
    usart_set_baudrate(dev, 9600);
    usart_set_databits(dev, 8);
    usart_set_parity(dev, USART_PARITY_NONE);
    usart_set_stopbits(dev, USART_CR2_STOPBITS_1);
    usart_set_mode(dev, USART_MODE_TX_RX);
    usart_set_flow_control(dev, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(dev);

    cookie_io_functions_t stub = { _iord, _iowr, NULL, NULL };
    FILE *fp = fopencookie((void *)dev, "rw+", stub);
    /* Do not buffer the serial line */
    setvbuf(fp, NULL, _IONBF, 0);
    return fp;
}

//##################

static void gpio_setup(void)
{
    // /* Setup GPIO pin GPIO8/9 on GPIO port C for LEDs. */
    // gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9);

    /* Setup GPIO pins for USART transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

    /* Setup USART TX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
}

//##################
static void clock_setup(void)
{
    /* Enable GPIOC clock for LED & USARTs. */
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Enable clocks for USART. */
    rcc_periph_clock_enable(RCC_USART1);
}
//##################

static void adc_setup(void)
{
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);

    //setup 0-3 channels of the ADC
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3);

	adc_power_off(ADC1);
	adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
	adc_calibrate(ADC1);
	adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);

	adc_enable_temperature_sensor();

	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++) {    /* Wait a bit. */
		__asm__("nop");
	}

}

// static void usart_setup(void)
// {
// 	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART1. */
// 	rcc_periph_clock_enable(RCC_USART1);
// 	rcc_periph_clock_enable(RCC_GPIOA);

// 	/* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
// 	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
// 	gpio_set_af(GPIOA, GPIO_AF1, GPIO9);

// 	/* Setup UART parameters. */
// 	usart_set_baudrate(USART1, 115200);
// 	usart_set_databits(USART1, 8);
// 	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
// 	usart_set_mode(USART1, USART_MODE_TX);
// 	usart_set_parity(USART1, USART_PARITY_NONE);
// 	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

// 	/* Finally enable the USART. */
// 	usart_enable(USART1);
// }

// static void my_usart_print_int(uint32_t usart, int16_t value)
// {
// 	int8_t i;
// 	int8_t nr_digits = 0;
// 	char buffer[25];

// 	if (value < 0) {
// 		usart_send_blocking(usart, '-');
// 		value = value * -1;
// 	}

// 	if (value == 0) {
// 		usart_send_blocking(usart, '0');
// 	}

// 	while (value > 0) {
// 		buffer[nr_digits++] = "0123456789"[value % 10];
// 		value /= 10;
// 	}

// 	for (i = nr_digits-1; i >= 0; i--) {
// 		usart_send_blocking(usart, buffer[i]);
// 	}

// 	usart_send_blocking(usart, '\r');
// 	usart_send_blocking(usart, '\n');
// }

int main(void)
{
    uint16_t temp;
    // int i, c = 0;
    FILE *fp;

    clock_setup();
    gpio_setup();
    fp = usart_setup(USART1);

	adc_setup();

	while (1) {
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)));

		temp = adc_read_regular(ADC1);
        // my_usart_print_int(USART1, temp);

		int i;
		for (i = 0; i < 800000; i++) {   /* Wait a bit. */
			__asm__("nop");
		}
	}

	return 0;
}

