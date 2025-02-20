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
#include <sys/types.h>

#include "led.h"

//
#define BKPT asm("bkpt 255")

//
#define TXBUFFERSIZE 16                                 //uint16_t for the uart in the future!
#define RXBUFFERSIZE TXBUFFERSIZE

void adc_isr(void);

//
uint8_t channel_array[] = { 0, 1, 2, 3, ADC_CHANNEL_TEMP, ADC_CHANNEL_VREF };    //readme 1, 1, TEMP
// uint8_t channel_array[] = { ADC_CHANNEL_VREF, ADC_CHANNEL_VBAT };    //readme 1, 1, TEMP
//
uint8_t aTxBufferPos = 0;                           //usage as semaphore
uint8_t aTxBuffer[TXBUFFERSIZE];

//
uint8_t aRxBufferPos = 0;
uint8_t aRxBuffer[RXBUFFERSIZE];

//
uint8_t cnt = 1;
//
float testf = -9999.00030f;
int8_t testfsign = 1;

uint8_t adcWCP;             //channel in process
uint32_t adcCHA[6];         //

/**
  * @brief
  */
FILE *fp;
static ssize_t _iord(void *_cookie, char *_buf, size_t _n);
static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n);
void transmitBuffer(void);


/* Temperature sensor calibration value address */
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
#define VDD_CALIB ((uint16_t) (330))
#define VDD_APPLI ((uint16_t) (330))

/**
  * @brief
  * @retval
  */
void adc_isr(void)
{
    BKPT;
}


/**
  * @brief just copy paste
  * @retval
  */
static void adc2_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC);
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

    adc_power_off(ADC1);
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);

    adc_calibrate(ADC1);
    adc_calibrate_wait_finish(ADC1);

    adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);
    //
    adc_enable_temperature_sensor();
    adc_enable_vrefint();
    adc_enable_vbat_sensor();
    //
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
    // adc_set_regular_sequence(ADC1, 7, channel_array);
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
        allsegmentsoff();
        float2led(testf);
        // dot(cnt++%8, 1);      //movement of dots
        // aTxBuffer[7]=cnt++;      //movement of the leds above
        transmitBuffer();
        // BKPT;
    }

    // //buttons count up
    // if (aRxBuffer[0] == 0x01) {
    //     testfsign = 1;
    // }
    // //buttons count down
    // if (aRxBuffer[0] == 0x02) {
    //     testfsign = -1;
    // }
    //adc channel 0
    if (aRxBuffer[0] == ( 1 << 7 ) ) {
        adcWCP = channel_array[0];
        testf = adcCHA[adcWCP];
        aTxBuffer[7] = 0x80;
    }
    //adc channel 1
    if (aRxBuffer[0] == ( 1 << 6 ) ) {
        adcWCP = channel_array[1];
        testf = adcCHA[adcWCP];
        aTxBuffer[7] = 0x40;
    }
    //adc channel 2
    if (aRxBuffer[0] == ( 1 << 5 ) ) {
        adcWCP = channel_array[2];
        testf = adcCHA[adcWCP];
        aTxBuffer[7] = 0x20;
    }
    //adc channel 3
    if (aRxBuffer[0] == ( 1 << 4 ) ) {
        adcWCP = channel_array[3];
        testf = adcCHA[adcWCP];
        aTxBuffer[7] = 0x10;
    }
    //adc channel 4
    if (aRxBuffer[0] == ( 1 << 3 ) ) {
        adcWCP = channel_array[4];
        // testf = adcCHA[4];

        testf = (((int32_t) adcCHA[4] * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR );     //PAGE 954
        testf *= (int32_t)(110 - 30);
        testf /= (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
        testf += 30.0f;

        // testf = ( (int32_t)(110-30) / (int32_t) (*TEMP110_CAL_ADDR-*TEMP30_CAL_ADDR)) * (int32_t) (adcCHA[4]-*TEMP30_CAL_ADDR) + 30.0f;
        // testf = VDD_CALIB;
        aTxBuffer[7] = 0x08;
    }
    //adc channel 5
    if (aRxBuffer[0] == ( 1 << 2 ) ) {
        adcWCP = channel_array[5];
        testf = adcCHA[5];
        aTxBuffer[7] = 0x04;
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
    fp = fopencookie((void *)dev, "rw+", stub);
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
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO8);
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

    rcc_apb1_frequency = 48000000;
    rcc_ahb_frequency = 48000000;

    /* Enable GPIOC clock for LED, ADC & USARTs. */
    rcc_periph_clock_enable(RCC_GPIOA);         //adc PA0-PA3
    rcc_periph_clock_enable(RCC_GPIOB);         //leds on PB0 PB1. PB3 MCO

    /* Enable clocks for USART. */
    rcc_periph_clock_enable(RCC_USART1);

    //MCO stuff for debugging serial speed
    // rcc_set_mco( RCC_CFGR_MCO_HSE  );      //source for mco and prescaler. helper function with 24 bits shift
    // rcc_set_mco( RCC_CFGR_MCO_PLL );      //source for mco and prescaler. helper function with 24 bits shift. can not set div/1/2
    // rcc_set_mco( RCC_CFGR_MCO_SYSCLK );
    rcc_set_mco( RCC_CFGR_MCO_HSI14 );
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

    adc_power_off(ADC1);                //Ensure that ADEN = 0 and DMAEN = 0. dali da chakame za adcen == 0 ?
    ADC1_CFGR1 &= ~ADC_CFGR1_DMAEN;

    adc_calibrate(ADC1);
    // adc_calibrate_wait_finish(ADC1);            //Vch = (3.3V * VREFINT_CAL * ADC_DATA) / (VREFINT_DATA * FULL_SCALE)   ...todo readme

    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);            //readme for HSI CLK ADC calibrate

    adc_set_operation_mode(ADC1, ADC_MODE_SEQUENTIAL);

    //
    // ADC1_SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;

    // ADC1_IER = ADC_IER_EOCIE | ADC_IER_EOSEQIE | ADC_IER_OVRIE;
    adc_enable_eoc_interrupt(ADC1);
    adc_enable_eoc_sequence_interrupt(ADC1);
    adc_enable_overrun_interrupt(ADC1);

    // adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);

    adc_enable_temperature_sensor();

    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
    // adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1);

    /* Wait for ADC starting up. */  //fixme.. more accurate waiting for interrupt
	int i;
	for (i = 0; i < 800000; i++) {    /* Wait a bit. */
		__asm__("nop");
	}
    nvic_enable_irq(NVIC_ADC_COMP_IRQ);
    nvic_set_priority(NVIC_ADC_COMP_IRQ,0);
}


static uint16_t read_adc_naiive(uint8_t channel)
{
    uint8_t channel_array[16];
    channel_array[0] = channel;
    adc_set_regular_sequence(ADC1, 1, channel_array);
    adc_start_conversion_regular(ADC1);
    while (!adc_eoc(ADC1));
    uint16_t reg16 = adc_read_regular(ADC1);
    return reg16;
}

/**
  * @brief
  * @retval
  */
void transmitBuffer(void)
{
    for (uint8_t i=0; i<10; i++) {
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
    // uint16_t temp;

    clock_setup();
    gpio_setup();
    fp = usart_setup(USART1);       //
    adc2_setup();

    allsegmentsoff();
    brightness(0xf4);

    // BKPT;

	while (1) {
        int i;
        for(i=0;i<6;i++)
        {
            if (i == 0) {
                adcCHA[i] = read_adc_naiive(i);
            } else if (i == 1) {
                adcCHA[i] = read_adc_naiive(i);
            } else if (i == 2) {
                adcCHA[i] = read_adc_naiive(i);
            } else if (i == 3) {
                adcCHA[i] = read_adc_naiive(i);
            } else if (i == 4) {
                adcCHA[i] = read_adc_naiive(ADC_CHANNEL_TEMP);
            } else if (i == 5) {
                adcCHA[i] = read_adc_naiive(ADC_CHANNEL_VREF);
            } else {
                BKPT;
            }
        }

		for (i = 0; i < 800000; i++) {   /* Wait a bit. */
			__asm__("nop");
		}
        // testf = temp;
	}

	return 0;
}

/**
  * @brief
  * @retval
  */
void _close(void)
{
    BKPT;
}

/**
  * @brief
  * @retval
  */
void _lseek(void)
{
    BKPT;
}

/**
  * @brief
  * @retval
  */
void _read(void)
{
    BKPT;
}

/**
  * @brief
  * @retval
  */
void _write(void)
{
    BKPT;
}

/**
  * @brief
  * @retval
  */
void _fstat_r(void)
{
    BKPT;
}

/**
  * @brief
  * @retval
  */
void _exit(void)
{
    BKPT;
}

/**
  * @brief
  * @retval
  */
void _isatty_r(void)
{
    BKPT;
}

int _getpid(void)
{
  return 1;
}

int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}
