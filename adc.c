/*
 * POWER SORCE METER - chichko & smooker 2025
 *
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f0/nvic.h>
#include <libopencm3/stm32/f0/rcc.h>
#include <libopencm3/stm32/f0/flash.h>
#include <libopencm3/stm32/f0/gpio.h>

#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#include <sys/types.h>

#include "led.h"

//
#define BKPT asm("bkpt 255")
//
#define TXBUFFERSIZE 16                                 //uint16_t for the uart in the future!
#define RXBUFFERSIZE TXBUFFERSIZE

//
uint8_t channel_array[] = { 1, 1, ADC_CHANNEL_TEMP};

//UART bufers and positions
uint8_t aTxBufferPos = 0;
uint8_t aTxBuffer[TXBUFFERSIZE];
//
uint8_t aRxBufferPos = 0;
uint8_t aRxBuffer[RXBUFFERSIZE];
//
uint8_t cnt = 1;

/**
  * @brief
  */
FILE *fp;
static ssize_t _iord(void *_cookie, char *_buf, size_t _n);
static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n);

/**
  * @brief
  * @retval
  */
void usart1_isr(void)
{
    // USART interrupt and status register (USART_ISR)
    //IDLE OK!!!

    /* Check if we were called because of RXNE. */
    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) && ((USART_ISR(USART1) & USART_ISR_RXNE) != 0) && (((USART_ISR(USART1) & 0x07) == 0)) ) {
        aRxBuffer[0] = usart_recv(USART1);
        /* Enable transmit interrupt so it sends back the data. */
        USART_CR1(USART1) |= USART_CR1_TXEIE;
        gpio_toggle(GPIOB, GPIO0);
        gpio_toggle(GPIOB, GPIO1);
        hex2led(0x55aa55aa);
        dot(cnt++%8, 1);
        transmitBuffer();
        // BKPT;
    }

    if (aRxBuffer[0] != 0x00) {
        gpio_toggle(GPIOB, GPIO1);
        BKPT;
    }

    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) && ((USART_ISR(USART1) & USART_ISR_TXE) != 0)) {
         /* Disable the TXE interrupt as we don't need it anymore. */
         USART_CR1(USART1) &= ~USART_CR1_TXEIE;
    }
}

/**
  * @brief
  * @retval
  */
static ssize_t _iord(void *_cookie, char *_buf, size_t _n)
{
    /* dont support reading now */
    (void)_cookie;
    (void)_buf;
    (void)_n;
    return 0;
}

/**
  * @brief
  * @retval
  */
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

/**
  * @brief
  * @retval
  */
static FILE *usart_setup(uint32_t dev)
{
    /* Setup USART parameters. */
    usart_set_baudrate(dev, 9600);
    usart_set_databits(dev, 8);
    usart_set_parity(dev, USART_PARITY_NONE);
    usart_set_stopbits(dev, USART_CR2_STOPBITS_1);
    usart_set_mode(dev, USART_MODE_TX_RX);
    usart_set_flow_control(dev, USART_FLOWCONTROL_NONE);

    /* Enable USART1 Receive interrupt. */
    usart_enable_rx_interrupt(dev);        //not needed in DMA mode ??   below...

    // /* Enable USART1 Transmit interrupt. */
    // usart_enable_tx_interrupt(dev);        // rx enables tx interrupt

    /* Enable USART1 Receive interrupt. */
    // USART_CR1(USART1) |= USART_CR1_RXNEIE;      //same as above for enable rxie

    /* Finally enable the USART. */
    usart_enable(dev);

    nvic_set_priority(NVIC_USART1_IRQ, 0);      //checkme priority in the docs
    nvic_enable_irq(NVIC_USART1_IRQ);

    // //TX DMA only... int multiplexor
    // nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0);
    // nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

    /*
     *
     */
    cookie_io_functions_t stub = { _iord, _iowr, NULL, NULL };
    FILE *fp = fopencookie((void *)dev, "rw+", stub);
    /* Do not buffer the serial line */
    setvbuf(fp, NULL, _IONBF, 0);

    return fp;
}

/**
  * @brief
  * @retval
  */
static void gpio_setup(void)
{
    // /* Setup GPIO pin GPIO8/9 on GPIO port C for LEDs. */
    // gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9);

    /* Setup GPIO pins for USART transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

    /* Setup USART TX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9);

    /* Setup GPIO pins for USART receive. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);

    /* Setup USART RX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF1, GPIO10);


    //setup LEDS
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);

    //MCO
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO8);
    gpio_set_af(GPIOA, 0, GPIO8);

    gpio_set(GPIOB, GPIO0);
    gpio_set(GPIOB, GPIO1);
}

/**
  * @brief
  * @retval
  */
static void clock_setup(void)
{
    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);
    rcc_set_sysclk_source(RCC_HSE);

    rcc_set_hpre(RCC_CFGR_HPRE_NODIV);  // AHB Prescaler
    rcc_set_ppre(RCC_CFGR_PPRE_NODIV);  // APB1 Prescaler

    flash_prefetch_enable();
    flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);

    // PLL: 16MHz * 3 = 48MHz
    rcc_set_prediv(RCC_CFGR2_PREDIV_NODIV);
    rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL3);
    rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);
    rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);
    rcc_set_sysclk_source(RCC_PLL);

    // rcc_set_usbclk_source(RCC_PLL);

    rcc_apb1_frequency = 48000000;
    rcc_ahb_frequency = 48000000;

    /* Enable GPIOC clock for LED, ADC & USARTs. */
    rcc_periph_clock_enable(RCC_GPIOA);         //adc PA0-PA3
    rcc_periph_clock_enable(RCC_GPIOB);         //leds on PB0 PB1. PB3 MCO

    /* Enable clocks for USART. */
    rcc_periph_clock_enable(RCC_USART1);

    /* Enable DMA1 clock */
    // rcc_periph_clock_enable(RCC_DMA1);          //dma for USART TX only for now

    //MCO stuff for debugging serial speed
    // MCKOE

    // rcc_set_mco( RCC_CFGR_MCO_HSE  );      //source for mco and prescaler. helper function with 24 bits shift
    // rcc_set_mco( RCC_CFGR_MCO_PLL );      //source for mco and prescaler. helper function with 24 bits shift. can not set div/1/2
    rcc_set_mco( RCC_CFGR_MCO_SYSCLK );

#pragma message(VALUE(RCC_MCO_NODIV))
}

/**
  * @brief
  * @retval
  */
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

/**
  * @brief
  * @retval
  */
void transmitBuffer()
{
    for (uint8_t i=0; i<8; i++) {
        // usart_send_blocking(USART1, aTxBuffer[i]);
        fprintf(fp, "%c", aTxBuffer[i]);
    }
}

/**
  * @brief
  * @retval
  */
int main(void)
{
    uint16_t temp;

    clock_setup();
    gpio_setup();
    fp = usart_setup(USART1);

	adc_setup();

    allsegmentsoff();

    // BKPT;

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

