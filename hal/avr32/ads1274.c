/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file ads1274.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the Driver for the external ADC: ADS1274  
 * (4ch, 24 bit, SPI interface)
 *
 ******************************************************************************/


#include "ads1274.h"
#include "gpio.h"
#include "tc.h"
#include "scif_uc3c.h"
#include "genclk.h"
#include "eic.h"
#include "intc.h"
#include "led.h"
#include "spi_master.h"
#include "spi_buffered.h"

#include "dac_dma.h"

#define ADC_MODE0 AVR32_PIN_PB03			///< Define the pin selecting the ADC mode 
#define ADC_MODE1 AVR32_PIN_PB04			///< Define the pin selecting the ADC mode 

#define ADC_FORMAT0 AVR32_PIN_PB02			///< Define the pin selecting the ADC format 
#define ADC_FORMAT1 AVR32_PIN_PB01			///< Define the pin selecting the ADC format 
#define ADC_FORMAT2 AVR32_PIN_PB00			///< Define the pin selecting the ADC format 

#define ADC_PWDN1 AVR32_PIN_PC00			///< Define the pin selecting the ADC configuration
#define ADC_PWDN2 AVR32_PIN_PC01			///< Define the pin selecting the ADC configuration
#define ADC_PWDN3 AVR32_PIN_PC02			///< Define the pin selecting the ADC configuration
#define ADC_PWDN4 AVR32_PIN_PC03			///< Define the pin selecting the ADC configuration


#define ADC_DREADY AVR32_PIN_PD21			///< Define the pin selecting the ADC ready


#define ADC_CLOCK_PIN AVR32_TC1_B0_PIN		///< Define the pin selecting the ADC clock pin
#define ADC_CLKDIV	AVR32_PIN_PB05			///< Define the pin selecting the ADC clock divider

#define EXAMPLE_TC                    (&AVR32_TC1)
#define EXAMPLE_TC_CHANNEL_ID         0
#define EXAMPLE_TC_CHANNEL_PIN        AVR32_TC1_B0_0_0_PIN			
#define EXAMPLE_TC_CHANNEL_FUNCTION   AVR32_TC1_B0_0_0_FUNCTION

/**
 * \brief Declare an object containing the structure of wave form(?)
 */
  tc_waveform_opt_t waveform_opt =
  {
    .channel  = EXAMPLE_TC_CHANNEL_ID,		///< Channel selection.
    .bswtrg   = TC_EVT_EFFECT_NOOP,			///< Software trigger effect on TIOB.
    .beevt    = TC_EVT_EFFECT_NOOP,			///< External event effect on TIOB.
    .bcpc     = TC_EVT_EFFECT_SET,			///< RC compare effect on TIOB.
    .bcpb     = TC_EVT_EFFECT_CLEAR,		///< RB compare effect on TIOB.
    .aswtrg   = TC_EVT_EFFECT_NOOP,			///< Software trigger effect on TIOA.
    .aeevt    = TC_EVT_EFFECT_NOOP,         ///< External event effect on TIOA.
    .acpc     = TC_EVT_EFFECT_NOOP,         ///< RC compare effect on TIOA: toggle.
    .acpa     = TC_EVT_EFFECT_NOOP,         ///< RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

    .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,      ///< Waveform selection: Up mode without automatic trigger on RC compare.
    .enetrg   = false,                        ///< External event trigger enable.
    .eevt     = TC_EXT_EVENT_SEL_XC0_OUTPUT,  ///< External event selection.
    .eevtedg  = TC_SEL_NO_EDGE,               ///< External event edge selection.
    .cpcdis   = false,                        ///< Counter disable when RC compare.
    .cpcstop  = false,                        ///< Counter clock stopped with RC compare.

    .burst    = TC_BURST_NOT_GATED,           ///< Burst signal selection.
    .clki     = TC_CLOCK_RISING_EDGE,         ///< Clock inversion.
    .tcclks   = TC_CLOCK_SOURCE_TC2           ///< Internal source clock 2, connected to fPBA / 2.
 };
  
volatile avr32_tc_t *tc = EXAMPLE_TC;
struct genclk_config gcfg;
static volatile int32_t adc_buffer[ADC_INPUT_CHANNELS][ADC_BUFFER_SIZE];
static volatile int32_t sample_counter;
static volatile int32_t interrupt_counter;
static volatile generatorfunction function_generator;

#define EXT_INT_LINES 1
eic_options_t eic_options[EXT_INT_LINES];  

///< Function prototype definitions
void process_data(void);
__attribute__((__naked__)) void eic_nmi_handler( void );

int32_t get_interrupt_counter(void) 
{
	return interrupt_counter;
}


void process_data(void) 
{
	int32_t value;
	if (sample_counter >= ADC_BUFFER_SIZE) 
	{
		return;
	}	
	uint8_t* buffer = spi_buffered_get_spi_in_buffer(ADC_SPI_INDEX);
	for (int32_t ch = 0; ch < 4; ch++) 
	{
		value = (buffer[3 * ch] << 24) + (buffer[3 * ch + 1] << 16) + (buffer[3 * ch + 2] << 8);
		adc_buffer[ch][sample_counter] = (value/256);
		
	}
	
	if (function_generator != NULL) 
	{
		dac_dma_set_value((*function_generator)(sample_counter));
	}		
	sample_counter++;
}  

void ads1274_set_DAC_generator_function(generatorfunction new_function_generator ) 
{
	function_generator = new_function_generator;
}

float get_sample(int32_t channel, int32_t sample) 
{
	return adc_buffer[channel][sample];
}

int32_t get_sampling_status(void) 
{
	return sample_counter;
}

///< Initializes ADC (configures Pins, starts Clock, sets defaults)
void ads1274_init_DAC(void) 
{
	function_generator = NULL;
	
	///< set mode to "high resolution"
	gpio_configure_pin(ADC_MODE0,GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);	
	gpio_configure_pin(ADC_MODE1,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);	
	
	///< set Format to Fixed-position TDM via SPI
	gpio_configure_pin(ADC_FORMAT0,GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);	
	gpio_configure_pin(ADC_FORMAT1,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);	
	gpio_configure_pin(ADC_FORMAT2,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);	
	
	///< configure the four channels
	gpio_configure_pin(ADC_PWDN1,GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);	
	gpio_configure_pin(ADC_PWDN2,GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);	
	gpio_configure_pin(ADC_PWDN3,GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);	
	gpio_configure_pin(ADC_PWDN4,GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);	

	//gpio_configure_pin(AVR32_TC1_B0_0_0_PIN,GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);	
	//gpio_configure_pin(AVR32_PIN_PC19, GPIO_DIR_OUTPUT| GPIO_INIT_HIGH);	

	ads1274_ADC_switch_clock(true);	
	
	//tc_init_waveform(tc, &waveform_opt);  ///< Initialize the timer/counter .

	///< Set the compare triggers.
	//tc_write_rb(tc, EXAMPLE_TC_CHANNEL_ID, 0x1);     ///< Set RA value.
	//tc_write_rc(tc, EXAMPLE_TC_CHANNEL_ID, 0x2);     ///< Set RC value.
	
	///< Start the timer/counter.
	//tc_start(tc, EXAMPLE_TC_CHANNEL_ID);
	
	///< Enable edge-triggered interrupt.
	eic_options[0].eic_mode   = EIC_MODE_EDGE_TRIGGERED;
	///< Interrupt will trigger on falling edge.
	eic_options[0].eic_edge  = EIC_EDGE_FALLING_EDGE;
	///< Initialize in synchronous mode : interrupt is synchronized to the clock
	eic_options[0].eic_async  = EIC_SYNCH_MODE;
	///< Set the interrupt line number.
	eic_options[0].eic_line   = EXT_NMI;
	
	gpio_enable_module_pin(AVR32_EIC_EXTINT_0_1_PIN, AVR32_EIC_EXTINT_0_1_FUNCTION);
	
	///<Disable_global_interrupt();
	///< Initialize interrupt vectors.
	
	eic_init(&AVR32_EIC, eic_options, 1);

	///<INTC_init_interrupts();
	///< initialize SPI0 interface
	spi_buffered_init(&AVR32_SPI0, ADC_SPI_INDEX);
	spi_buffered_init_DMA(0, 12);
	spi_buffered_set_callback(ADC_SPI_INDEX, &process_data);
	
	///< Register the EIC interrupt handlers to the interrupt controller.
	//INTC_register_interrupt(&eic_int_handler1, AVR32_EIC_IRQ_1, AVR32_INTC_INT1);
	///< Enable the chosen lines and their corresponding interrupt feature.
	eic_enable_line(&AVR32_EIC, eic_options[0].eic_line);
	eic_enable_interrupt_line(&AVR32_EIC, eic_options[0].eic_line);
	
	///< Enable_global_interrupt();
	eic_clear_interrupt_line(&AVR32_EIC, EXT_NMI);
	
	///< activate sync and clkdiv
	gpio_configure_pin(ADC_CLKDIV,GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);	
}

///< Enable/Disable the clock to the ADC
void ads1274_ADC_switch_clock(bool on_off) 
{
	if (on_off == true) 
	{
		gpio_enable_module_pin(AVR32_SCIF_GCLK_1_1_PIN, AVR32_SCIF_GCLK_1_1_FUNCTION);	

		gpio_configure_pin(ADC_CLKDIV,GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);	
		
		//scif_gc_setup(AVR32_SCIF_GCLK_GCLK0PIN, SCIF_GCCTRL_CPUCLOCK, 1, 1);
		//scif_gc_enable(AVR32_SCIF_GCLK_GCLK0PIN);
		genclk_config_defaults(&gcfg, AVR32_SCIF_GCLK_GCLK1PIN);
		genclk_config_set_source(&gcfg, GENCLK_SRC_PLL1);
		genclk_config_set_divider(&gcfg, 2);
		genclk_enable(&gcfg, AVR32_SCIF_GCLK_GCLK1PIN);
	} 
	else 
	{
		genclk_disable(AVR32_SCIF_GCLK_GCLK1PIN);	
	}
}

///< starts sampling, captures one buffer length and then stops
void ads1274_ADC_start_oneshot(void)
{
	//Disable_global_interrupt();
	//eic_enable_interrupt_line(&AVR32_EIC, eic_options[0].eic_line);
	sample_counter = 0;
	//Enable_global_interrupt();
}

/* NOT IMPLEMENTED YET
///< Switch the four input channels on or off
void ads1274_ADC_switch_channel(int32_t channel, bool on_off){};
	
///< configures the ADC mode (refer to datasheet for options)
void ads1274_ADC_set_mode(int32_t mode){};

///< enables continuous sampling  -- not implemented yet
//void ads1274_ADC_start_sampling(void){};

///< stops sampling immediately
void ads1274_ADC_stop_sampling(void){};

///< Returns true if one-shot sampling has finished
bool Sampling_Complete(void){};
*/

__attribute__((__naked__))
void eic_nmi_handler( void )
{
	//int32_t i = 0;
	__asm__ __volatile__ (
			/* Save registers not saved upon NMI exception. */
			"pushm   r0-r12, lr\n\t"
			);
	//interrupt_counter++;
	
	if (sample_counter < ADC_BUFFER_SIZE) 
	{
		spi_buffered_trigger_DMA(0, 12);
	} 
	else 
	{
		//eic_disable_interrupt_line(&AVR32_EIC,eic_options[0].eic_line);
	}
	eic_clear_interrupt_line(&AVR32_EIC, EXT_NMI);
	__asm__ __volatile__ (
			/* Restore the registers. */
			"popm   r0-r12, lr\n\t"
			/* Leaving the exception handler. */
			"rete"
			);
}
 