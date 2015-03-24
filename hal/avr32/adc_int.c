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
 * \file adc_int.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the Analog to Digital Converter module using interruptions
 *
 ******************************************************************************/


#include "adc_int.h"
#include "gpio.h"
#include "tc.h"
#include "intc.h"
#include "led.h"
#include <stdint.h>
#include "adcifa.h"
#include "time_keeper.h"
#include "dac_dma.h"
#include "sysclk.h"

#define ADC_INT_SEOS0 1			///< Define the Analog to Digital interrupt (?)
#define ADC_INT_SEOS1 16		///< Define the Analog to Digital interrupt (?)
#define ADC_INT_SEOC0 2			///< Define the Analog to Digital interrupt (?)
#define ADC_INT_SEOC1 32		///< Define the Analog to Digital interrupt (?)

///< GPIO pin/analog_monitor-function map.
static const gpio_map_t ADCIFA_GPIO_MAP = 
{
	{AVR32_ADCREF0_PIN,AVR32_ADCREF0_FUNCTION},
	{AVR32_ADCREFP_PIN,AVR32_ADCREFP_FUNCTION},
	{AVR32_ADCREFN_PIN,AVR32_ADCREFN_FUNCTION},
		
	{AVR32_ADCIN0_PIN, AVR32_ADCIN0_FUNCTION},
	{AVR32_ADCIN1_PIN, AVR32_ADCIN1_FUNCTION},
	{AVR32_ADCIN2_PIN, AVR32_ADCIN2_FUNCTION},
	{AVR32_ADCIN3_PIN, AVR32_ADCIN3_FUNCTION},
	{AVR32_ADCIN4_PIN, AVR32_ADCIN4_FUNCTION},
	{AVR32_ADCIN5_PIN, AVR32_ADCIN5_FUNCTION},
	{AVR32_ADCIN6_PIN, AVR32_ADCIN6_FUNCTION},
	{AVR32_ADCIN7_PIN, AVR32_ADCIN7_FUNCTION},
	{AVR32_ADCIN8_PIN, AVR32_ADCIN8_FUNCTION},
	{AVR32_ADCIN9_PIN, AVR32_ADCIN9_FUNCTION},
	{AVR32_ADCIN10_PIN, AVR32_ADCIN10_FUNCTION},
	{AVR32_ADCIN11_PIN, AVR32_ADCIN11_FUNCTION},
	{AVR32_ADCIN12_PIN, AVR32_ADCIN12_FUNCTION},
	{AVR32_ADCIN13_PIN, AVR32_ADCIN13_FUNCTION},
	{AVR32_ADCIN14_PIN, AVR32_ADCIN14_FUNCTION},
	{AVR32_ADCIN15_PIN, AVR32_ADCIN15_FUNCTION}

};

volatile avr32_adcifa_t *adcifa = &AVR32_ADCIFA;							///< ADCIFA IP registers address

static volatile int32_t sequencer_item_count, channel_count;					///< Declare counters
	
static volatile int32_t sample_counter, oversampling_counter;				///< Declare counters
	
static volatile int32_t number_of_samples, oversampling, oversampling_divider;	///< Declare ADC sampling stuff

bool continuous_mode;														///< Declare whether to work in continuous mode or not
	
static volatile uint32_t last_adc_int_time, adc_int_period;					///< Declare ADC interrupt time and period
	
///< 32bits version
// int32_t adci_buffer[ADCI_INPUT_CHANNELS][ADCI_BUFFER_SIZE];

///< 16bits version
static volatile int32_t internal_buffer[MAX_CHANNELS];		///< Declare an internal buffer
int16_t* adci_buffer[MAX_CHANNELS];							///< Declare a pointer on the ADC interrupt buffer
uint8_t even_odd;											///< Declare whether even or odd
	

///< ADC Configuration
volatile adcifa_opt_t adc_config_options = 
{
	.frequency                = ADC_FREQUENCY,		///< ADC frequency (Hz)
	.reference_source         = ADCIFA_REF1V,		///< Reference Source
	.sample_and_hold_disable  = false,				///< Disable Sample and Hold Time
	.single_sequencer_mode    = false,				///< Single Sequencer Mode
	.free_running_mode_enable = false,				///< Free Running Mode
	.sleep_mode_enable        = false				///< Sleep Mode
};
			
///< Sequencer Configuration
adcifa_sequencer_opt_t adcifa_sequence_opt = 
{
	.convnb               = 0,							///< Number of sequence
	.resolution           = ADCIFA_SRES_12B,			///< Resolution selection
	.trigger_selection    = ADCIFA_TRGSEL_ITIMER,		///< Trigger selection
	.start_of_conversion  = ADCIFA_SOCB_ALLSEQ,			///< Conversion Management
	.sh_mode              = ADCIFA_SH_MODE_STANDARD,	///< Oversampling Management
	.half_word_adjustment = ADCIFA_HWLA_NOADJ,			///< Half word Adjustment
	.software_acknowledge = ADCIFA_SA_NO_EOS_SOFTACK	///< Software Acknowledge
};
			
///< Conversions in the Sequencer Configuration
adcifa_sequencer_conversion_opt_t adcifa_sequencer0_conversion_opt[SLOTS_PER_SEQUENCER];
	
__attribute__((__interrupt__))
static void process_data(void) 
{
	int32_t ch;
	volatile int16_t value;

	if (sample_counter>=number_of_samples)  
	{
		if (continuous_mode) 
		{
			sample_counter=0;
			oversampling_counter=0;
		} else 
		{
			adcifa_disable_interrupt(adcifa, ADC_INT_SEOS0);
			//adcifa_disable_interrupt(adcifa, ADC_INT_SEOS1);
			adcifa_stop_itimer(adcifa);
		}
	} 
	else 
	{
		if ((adcifa->sr&ADC_INT_SEOS0) ==0)
		{}
		//|| ((adcifa->sr&ADC_INT_SEOS1) ==0) ) {}
		else 
		{
			adc_int_period=(time_keeper_get_time_ticks() - last_adc_int_time);
			last_adc_int_time=time_keeper_get_time_ticks();
		
			if (sample_counter>=0) 
			{
				if (oversampling_counter<=0) 
				{
					for (ch=0; ch<sequencer_item_count; ch++) 
					{
						value=adcifa->resx[ch];
						internal_buffer[ch]=  value ;
					}
				}
				else 
				{			
					for (ch=0; ch<sequencer_item_count; ch++) 
					{		
						value=adcifa->resx[ch];
						internal_buffer[ch]+= value ;
						//adci_buffer[ch][even_odd][sample_counter]+=value;
					}			
				}
			}	
			else 
			{
				sample_counter++; 
				return;
			}
			//if (function_generator!= NULL) {
			//	dac_dma_set_value((*function_generator)(sample_counter));
			//}
			oversampling_counter++;
	
			if (oversampling_counter >= oversampling) 
			{
				oversampling_counter=0;
				for (ch=0; ch<channel_count; ch++) 
				{
					int16_t *buffer = adci_buffer[ch];
					buffer[sample_counter] = internal_buffer[ch] / oversampling_divider;
				}
				sample_counter++;
			}		
			
			//dac_dma_set_value(even_odd * 400);
			///< acknowledge processing finished
			adcifa->scr=ADC_INT_SEOS0 | ADC_INT_SEOS1;
		}
	}
}


///< Initializes ADC (configures Pins, starts Clock, sets defaults)
void adc_int_init(uint32_t adc_frequency, uint8_t reference_source)
{
	///< Assign and enable GPIO pins to the ADC function.
	gpio_enable_module(ADCIFA_GPIO_MAP, sizeof(ADCIFA_GPIO_MAP) / sizeof(ADCIFA_GPIO_MAP[0]));

	adc_config_options.frequency=adc_frequency;
	adc_config_options.reference_source=reference_source;

	////</ Get ADCIFA Factory Configuration
	adcifa_get_calibration_data(adcifa, (adcifa_opt_t *)&adc_config_options);
	if ((uint16_t)adc_config_options.offset_calibration_value == 0xFFFF)
	{
		///< Set default calibration if Engineering samples and part is not programmed
		adc_config_options.offset_calibration_value = 0x3B;
		adc_config_options.gain_calibration_value = 0x4210;
		adc_config_options.sh0_calibration_value = 0x210;
		adc_config_options.sh1_calibration_value = 0x210;
	}
	adc_config_options.offset_calibration_value = 0x3B; ///< offset correction

	///< Configure ADCIFA core
	adcifa_configure(adcifa, (adcifa_opt_t *)&adc_config_options, sysclk_get_peripheral_bus_hz((const volatile void *)AVR32_ADCIFA_ADDRESS));

	adc_int_clear_sequencer();
	continuous_mode=false;
	///< Configure ADCIFA sequencer 1
	//adcifa_configure_sequencer(adcifa, 1, &adcifa_sequence_opt, adcifa_sequencer1_conversion_opt);
		
	adcifa_disable_interrupt(adcifa, 0xffffffff);
	INTC_register_interrupt( (__int_handler) &process_data, AVR32_ADCIFA_SEQUENCER0_IRQ, AVR32_INTC_INT1);
	//INTC_register_interrupt( (__int_handler) &process_data, AVR32_ADCIFA_SEQUENCER1_IRQ, AVR32_INTC_INT1);
	//int32_t period_us=1000000 / samplingrate;
}


void adc_int_clear_sequencer(void) 
{
	sequencer_item_count=0;
	adcifa_sequence_opt.convnb = sequencer_item_count;
}


int8_t adc_int_sequencer_add(int16_t* buffer, uint8_t input_p, uint8_t input_n, uint8_t gain) 
{	
	if (sequencer_item_count<SLOTS_PER_SEQUENCER - 1) 
	{
		adcifa_sequencer0_conversion_opt[sequencer_item_count].channel_p = input_p;
		adcifa_sequencer0_conversion_opt[sequencer_item_count].channel_n = input_n;
		adcifa_sequencer0_conversion_opt[sequencer_item_count].gain = gain;
		
		adci_buffer[sequencer_item_count] = buffer;
		sequencer_item_count++;
		adcifa_sequence_opt.convnb = sequencer_item_count;
		return sequencer_item_count;
	} 
	else 
	{
		return -1;
	}
}


///< starts sampling, captures one buffer length and then stops
void adc_int_start_sampling(int32_t length, int32_t samplingrate, int32_t set_oversampling, int32_t set_oversampling_divider, bool continuous)
{
	///< Configure ADCIFA sequencer 0
	adcifa_sequence_opt.convnb = sequencer_item_count;
	adcifa_configure_sequencer(adcifa, 0, (adcifa_sequencer_opt_t *)&adcifa_sequence_opt, (adcifa_sequencer_conversion_opt_t *)&adcifa_sequencer0_conversion_opt);
	
	oversampling = set_oversampling;
	oversampling_divider = set_oversampling_divider;

	volatile int32_t period_us = adc_config_options.frequency / (samplingrate*oversampling);	
	oversampling_counter = 0;
	sample_counter = -10;
	number_of_samples = length;
	continuous_mode = continuous;
	channel_count = sequencer_item_count;

	adcifa_enable_interrupt(adcifa, ADC_INT_SEOS0);
	//adcifa_enable_interrupt(adcifa, ADC_INT_SEOS1);
	adcifa_start_itimer(adcifa, period_us);
}


///< stops sampling immediately
void adc_int_stop_sampling(void)
{
	adcifa_stop_itimer(adcifa);	
}


///< Returns true if one-shot sampling has finished
bool adc_int_sampling_complete(void)
{
	return (sample_counter>=number_of_samples);
}


//void ads1274_set_DAC_generator_function(generatorfunction new_function_generator ) {
//	function_generator=new_function_generator;
	
//}


int16_t adc_int_get_sample(int32_t channel, int32_t sample) 
{
	int16_t *buffer=adci_buffer[channel];
	return buffer[sample];
}


int16_t** adc_int_get_buffer(void) 
{
	return adci_buffer;
}

	
int32_t adc_int_get_sampling_status(void) 
{
	return sample_counter;
}


uint32_t adc_int_get_period(void) 
{
	return adc_int_period;
}