/*
 * POWER SORCE METER - chichko & smooker 2025
 *
 */


#include <libopencm3/stm32/f0/nvic.h>
#include <libopencm3/stm32/f0/rcc.h>
#include <libopencm3/stm32/f0/flash.h>
#include <libopencm3/stm32/f0/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
// #include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
// #include <libopencm3/stm32/iwdg.h>

#include <stdio.h>
#include <sys/types.h>

#include "led.h"


#ifdef ENABLE_SEMIHOSTING

#if ENABLE_SEMIHOSTING == 1
#pragma message ( "C Preprocessor got here!" )
extern void initialise_monitor_handles(void);
#endif
#endif

//
#define BKPT asm("bkpt 255")

//
#define TXBUFFERSIZE 16                                 //uint16_t for the uart in the future!
#define RXBUFFERSIZE TXBUFFERSIZE

//
uint8_t channel_array[] = { 0, 1, 2, 3, ADC_CHANNEL_TEMP, ADC_CHANNEL_VREF };    //readme 1, 1, TEMP

//
uint8_t aTxBufferPos = 0;                           //usage as semaphore
uint8_t aTxBuffer[TXBUFFERSIZE];

//
uint8_t aRxBufferPos = 0;
uint8_t aRxBuffer[RXBUFFERSIZE];

//
float testf = -9999.90030f;     //float to display

//
float arrf[200];             //array of floats for average
uint8_t arri = 0;

//
uint8_t adcWCP = 0;             //channel in process
uint32_t adcCHA[6];         // 0 - 3, TEP, VREF
uint8_t ledCHA = 4;          //which channel to display 4 = temp, 5 = vref

/**
  * @brief
  */
FILE *fp;
static ssize_t _iord(void *_cookie, char *_buf, size_t _n);
static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n);
static void transmitBuffer(void);


/* Temperature sensor calibration value address */
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))

// Vref = Vdd
#define VDD_CALIB ((uint16_t) (330))
#define VDD_APPLI ((uint16_t) (325))    //measured 3.252V

//voltage coeff
float vcoeff = (float) VDD_APPLI / (float) VDD_CALIB;

//DECLARATIONS
static void read_adc_naiive(uint8_t channel);
static void dma_write(char *data, int size);
static void tim_setup(void);
static void nvic_setup(void);

// void dma1_channel4_7_dma2_channel3_5_isr(void);
// void dma1_channel1_isr(void);
// void dma1_channel2_3_dma2_channel1_2_isr(void);

/**
  * @brief
  * @retval
  */
void sys_tick_handler(void)
{

}

/**
  * @brief
  * @retval
  */
void adc_comp_isr(void)
{
    if ( (ADC1_ISR & ADC_ISR_EOSEQ) > 0 ) {
        gpio_clear(GPIOB, GPIO1);
        // BKPT;
        adcCHA[adcWCP] = ADC1_DR;
        adc_clear_eoc_sequence_flag(ADC1);
        gpio_clear(GPIOB, GPIO0);
    }
}

/**
  * @brief
  * @retval
  */
static float averagef(void)
{
    int i;
    float result = 0.0f;

    for (i=0;i<200;i++) {
        result += arrf[i];
    }
    return result / 200.0f;
}

/**
  * @brief
  * @retval
  */
static void pushf(float input)
{
    if (arri >= 200)
        arri = 0;
    arrf[arri++] = input;
    if (arri >= 200)
        arri = 0;
}

/**
  * @brief just copy paste
  * @retval
  */
static void adc2_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC);
    rcc_periph_clock_enable(RCC_GPIOA);

    //
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3);

    //
    adc_power_off(ADC1);
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);

    //
    adc_calibrate(ADC1);

    //
    adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
    // adc_disable_external_trigger_regular(ADC1);
    // adc_enable_external_trigger_regular(ADC1, )
    adc_set_right_aligned(ADC1);

    //
    adc_enable_temperature_sensor();
    adc_enable_vrefint();
    adc_enable_vbat_sensor();

    //
    // tCONV = Sampling time + 12.5 x ADC clock cycles
    // Example:
    // With ADC_CLK = 14 MHz and a sampling time of 1.5 ADC clock cycles:
    // tCONV = 1.5 + 12.5 = 14 ADC clock cycles = 1 µs

    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_239DOT5);         //readme
    // adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);         //readme
    // adc_set_regular_sequence(ADC1, 7, channel_array);
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
    adc_disable_analog_watchdog(ADC1);
    adc_power_on(ADC1);

    /* Wait for ADC starting up. */
    while ( (ADC1_ISR & ADC_ISR_ADRDY) == 0 ) {
        __asm__("nop");
    }
    //
    adc_enable_eoc_interrupt(ADC1);
    adc_enable_eoc_sequence_interrupt(ADC1);

    // nvic_set_priority(NVIC_ADC_COMP_IRQ, 2);      //checkme priority in the docs
}

//priorities nvic page 216

void dma1_channel4_7_dma2_channel3_5_isr(void)
{
    BKPT;

}

void dma1_channel1_isr(void)
{
    BKPT;
}

static void nvic_setup(void)
{
    //todo.. priorities here
    nvic_enable_irq(NVIC_USART1_IRQ);
    nvic_enable_irq(NVIC_ADC_COMP_IRQ);
    nvic_enable_irq(NVIC_TIM14_IRQ);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);
}

static void tim_setup(void)
{
    //
    rcc_periph_clock_enable(RCC_TIM14);

    //check cortex timer system divisor!!!

    //
    // nvic_enable_irq(NVIC_TIM14_IRQ);     //migrated to nvic_setup
    //
    rcc_periph_reset_pulse(RST_TIM14);

    //PAGE 471

    //
    timer_set_mode(TIM14, TIM_CR1_CKD_CK_INT,
        TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_set_prescaler(TIM14, 1);

    /* Disable preload. */
    // timer_disable_preload(TIM14);
    // timer_enable_preload(TIM14);
    // timer_continuous_mode(TIM14);

    timer_set_counter(TIM14, 0);
    timer_clear_flag(TIM14, TIM_SR_UIF);
    nvic_clear_pending_irq(NVIC_TIM14_IRQ);

    timer_set_period(TIM14, 24000);                    //1ms

    timer_set_oc_value(TIM14, TIM_OC1, 1000);
    // timer_enable_oc_preload(TIM14, TIM_OC1);
    // timer_enable_oc_clear(TIM14, TIM_OC1);

    timer_enable_irq(TIM14, TIM_DIER_CC1IE);         //migrated to nvic_setup
    timer_enable_counter(TIM14);                     //migrated to nvic_setup
}

// void tim_stop(void)
// {
//     timer_disable_counter(TIM(TIM14)); // disable timer
//     timer_set_counter(TIM(TIM14), 0); // reset timer counter
//     timer_clear_flag(TIM(TIM14), TIM_SR_UIF); // clear timer flag
//     nvic_clear_pending_irq(NVIC_TIM_IRQ(TIM14)); // clear IRQ flag
// }

void tim14_isr(void)
{
    if (timer_get_flag(TIM14, TIM_SR_UIF)) {
        // timer_set_counter(TIM14, 0);

        /*
         * Get current timer value to calculate next
         * compare register value.
         */
        // uint16_t time = timer_get_counter(TIM14);

        /* Toggle LED to indicate compare event. */
        gpio_set(GPIOB, GPIO0);
        //start of ADC conversion
        if ( (ADC1_ISR & ADC_ISR_ADRDY) > 0 ) {
            if (adcWCP == 0) {
                read_adc_naiive(adcWCP);       // 0 - 3 channels
            } else if (adcWCP == 1) {
                read_adc_naiive(adcWCP);
            } else if (adcWCP == 2) {
                read_adc_naiive(adcWCP);
            } else if (adcWCP == 3) {
                read_adc_naiive(adcWCP);
            } else if (adcWCP == 4) {
                read_adc_naiive(ADC_CHANNEL_TEMP);  //
            } else if (adcWCP == 5) {
                read_adc_naiive(ADC_CHANNEL_VREF);
            } else {
                BKPT;
            }
        }
        timer_clear_flag(TIM14, TIM_SR_UIF);
        nvic_clear_pending_irq(NVIC_TIM14_IRQ);
        return;
    }
    if (timer_get_flag(TIM14, TIM_SR_CC1IF)) {
        timer_clear_flag(TIM14, TIM_SR_CC1IF);
        nvic_clear_pending_irq(NVIC_TIM14_IRQ);
        return;
    }
    timer_disable_counter(TIM14);
    BKPT;
}

/**
  * @brief
  * @retval
  */
void usart1_isr(void)
{
    //PAGE 744

    //ERROR HANDLING
    if ( (USART1_ISR & 0x07) != 0 ) {
        BKPT;
    }

    /* Check if we were called because of RXNE. */
    if (   (  (USART1_CR1 & USART_CR1_RXNEIE) != 0 )
         && ( (USART1_ISR & USART_ISR_RXNE) != 0 )
         && ( (USART1_ISR & 0x07) == 0) ) {

        // BKPT;

        aRxBuffer[0] = usart_recv(USART1);

        /* Enable transmit interrupt so it sends back the data. */
        USART_CR1(USART1) |= USART_CR1_TXEIE;
        gpio_toggle(GPIOB, GPIO0);       //debug only

        //adc channel 0
        if ( (aRxBuffer[0] == ( 1 << 7 )) | (ledCHA == 0) ) {
            pushf( ((adcCHA[0] * 10.0f ) / 4096.0f)*vcoeff );
            testf = averagef();
            aTxBuffer[7] = 0x80;
            ledCHA = 0;
        }

        //adc channel 1
        if ( (aRxBuffer[0] == ( 1 << 6 )) | (ledCHA == 1) ) {
            testf = adcCHA[1];
            aTxBuffer[7] = 0x40;
            ledCHA = 1;
        }

        //adc channel 2
        if ( (aRxBuffer[0] == ( 1 << 5 )) | (ledCHA == 2) ) {
            testf = adcCHA[2];
            aTxBuffer[7] = 0x20;
            ledCHA = 2;
        }

        //adc channel 3
        if ( (aRxBuffer[0] == ( 1 << 4 )) | (ledCHA == 3) ) {
            testf = adcCHA[3];
            aTxBuffer[7] = 0x10;
            ledCHA = 3;
        }

        //adc channel 4 - TEMPERATURE
        if ( (aRxBuffer[0] == ( 1 << 3 )) | (ledCHA == 4) ) {
            testf = (((int32_t) adcCHA[4] * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR );     //PAGE 954
            testf *= (int32_t)(110 - 30);
            testf /= (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
            testf += 30.0f;
            aTxBuffer[7] = 0x08;
            ledCHA = 4;
        }

        //adc channel 5 - VREF
        if ( (aRxBuffer[0] == ( 1 << 2 )) | (ledCHA == 5) ) {
            testf = adcCHA[5];
            aTxBuffer[7] = 0x04;
            ledCHA = 5;
        }

        //adc channel 6
        if (aRxBuffer[0] == ( 1 << 1 ) ) {
            //debug only
            testf = adcCHA[4];
            aTxBuffer[7] = 0x02;
        }

        /////////////////////////////////////////////
        /// \brief allsegmentsoff
        ///
        allsegmentsoff();
        float2led(testf);
        // dot(cnt++%8, 1);      //movement of dots
        // aTxBuffer[7]=cnt++;      //movement of the leds above
        // transmitBuffer();
        dma_write(aTxBuffer, 9);
        // BKPT;
        // nvic_clear_pending_irq(NVIC_USART1_IRQ);
        USART1_ISR &= ~USART_ISR_ORE;
        USART1_ISR &= ~USART_ISR_RXNE;
        // BKPT;
        return;
    }
    // BKPT;
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

    // nvic_set_priority(NVIC_USART1_IRQ, 4);      //checkme priority in the docs
    nvic_enable_irq(NVIC_USART1_IRQ);        //migrated to nvid_setup

    // //TX DMA only... int multiplexor
    nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);

    /*
     *
     */
    cookie_io_functions_t stub = { _iord, _iowr, NULL, NULL };
    fp = fopencookie((void *)dev, "rw+", stub);
    /* Do not buffer the serial line */
    setvbuf(fp, NULL, _IONBF, 0);

    return fp;
}


static void dma_write(char *data, int size)
{
    // BKPT;
    dma_channel_reset(DMA1, DMA_CHANNEL2);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t)&USART1_TDR);        //fixme. TDR ot DR
    dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t)data);
    dma_set_number_of_data(DMA1, DMA_CHANNEL2, size);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL2);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL2, DMA_CCR_PL_VERY_HIGH);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);
    dma_enable_channel(DMA1, DMA_CHANNEL2);
    usart_enable_tx_dma(USART1);
    // BKPT;
}

void dma1_channel2_3_dma2_channel1_2_isr(void)
{
    BKPT;
    if ((DMA1_ISR &DMA_ISR_TCIF1) != 0) {
        DMA1_IFCR |= DMA_IFCR_CTCIF1;

        // transfered = 1;
    }

    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);

    usart_disable_tx_dma(USART1);

    dma_disable_channel(DMA1, DMA_CHANNEL1);
}

// /*
//  * Set up timer to fire every x milliseconds
//  * This is a unusual usage of systick, be very careful with the 24bit range
//  * of the systick counter!  You can range from 1 to 2796ms with this.
//  */
// static void systick_setup(int xms)
// {
//     /* div8 per ST, stays compatible with M3/M4 parts, well done ST */
//     systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
//     /* clear counter so it starts right away */
//     STK_CVR = 0;

//     systick_set_reload(rcc_ahb_frequency / 8 / 1000 * xms);
//     systick_counter_enable();
//     systick_interrupt_enable();
// }

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
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO1);
    gpio_set(GPIOB, GPIO0);
    gpio_set(GPIOB, GPIO1);

    //MCO
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO8);
    gpio_set_af(GPIOA, 0, GPIO8);
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
static void read_adc_naiive(uint8_t channel)
{
    while ( (ADC1_ISR & ADC_ISR_ADRDY) == 0 ) {
        __asm__("nop");
    }
    adc_set_regular_sequence(ADC1, 1, channel_array);
    gpio_set(GPIOB, GPIO0);
    adc_start_conversion_regular(ADC1);
}

/**
  * @brief
  * @retval
  */
static void transmitBuffer(void)
{
    // for (uint8_t i=0; i<10; i++) {
    //     // usart_send_blocking(USART1, aTxBuffer[i]);
    //     fprintf(fp, "%c", aTxBuffer[i]);
    // }
    usart_enable_tx_dma(USART1);
}

/**
  * @brief
  * @retval
  */
void main(void)
{
    clock_setup();
    gpio_setup();


#if ENABLE_SEMIHOSTING == 1
#pragma message ( "C Preprocessor got here2!" );
    initialise_monitor_handles();
#endif

    // systick_setup(20);           //l8r

    adc2_setup();

    // iwdg_set_period_ms(25);
    // iwdg_start();

    // tim_setup();

    fp = usart_setup(USART1);       //

    nvic_setup();

    allsegmentsoff();
    brightness(0xf4);

    int i;
    for (i=0;i<32;i++) {
        arrf[i] = 0.0f;
    }

	while (1) {
        if (adcWCP != 0)
            BKPT;

        // iwdg_reset();
        // for (i = 0; i < 80000; i++) {   /* Wait a bit. */      //fixme
        //     __asm__("nop");
        // }
	}
    // go to somewhere - reset handler ?
}

// /**
//   * @brief
//   * @retval
//   */
// void _close(void)
// {
//     BKPT;
// }

// /**
//   * @brief
//   * @retval
//   */
// void _lseek(void)
// {
//     BKPT;
// }

// /**
//   * @brief
//   * @retval
//   */
// void _read(void)
// {
//     BKPT;
// }

// /**
//   * @brief
//   * @retval
//   */
// void _write(void)
// {
//     BKPT;
// }

// /**
//   * @brief
//   * @retval
//   */
// void _fstat_r(void)
// {
//     BKPT;
// }

// /**
//   * @brief
//   * @retval
//   */
// void _exit(int)
// {
//     BKPT;
//     while(1);
// }

// /**
//   * @brief
//   * @retval
//   */
// void _isatty_r(void)
// {
//     BKPT;
// }

// int _getpid(void)
// {
//   return 1;
// }

int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}
