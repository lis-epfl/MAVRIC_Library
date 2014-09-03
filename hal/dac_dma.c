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
 * \file dac_dma.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for the Internal DAC 
 * (Digital to Analog Conversion) using the DMA
 * 
 ******************************************************************************/


#include "dac_dma.h"
#include "board.h"
#include "gpio.h"
#include "power_clocks_lib.h"
#include "dacifb.h"
#include "intc.h"
#include "pevc.h"
#include "pdca.h"
#include "print_util.h"

static volatile uint16_t* buffer;					///< pointer to the sampling buffer of the DAC
static volatile uint16_t from, to;					///< (?)
static volatile int32_t autoplay;						///< (?)
volatile avr32_pevc_t   *ppevc  = &AVR32_PEVC;		///< (?)
volatile avr32_dacifb_t *dacifb = &AVR32_DACIFB0;	///< DACIFB registers address
S16 dac_value_audio = -1;							///< (?)
U8 dac_channel_audio = DAC_AUDIO_CHANNEL;			///< Assign the on-board sensors to their DAC channel.

///< Functions Prototype Definition
void init_pevc(void);
void init_gclk(void);
void DAC_pause(void);
void DAC_resume(void);
int32_t  DAC_is_finished(void);

/**
 * \brief intterupt handler for the DAC
*/
__attribute__((__interrupt__))
static void pdca_int_handler_dac(void)
{
	AVR32_PDCA.channel[PDCA_CHANNEL_DAC].isr;
	if (autoplay == 0) 
	{
		pdca_disable(PDCA_CHANNEL_DAC);
	    pdca_disable_interrupt_transfer_complete(PDCA_CHANNEL_DAC);
	}	
	else 
	{	
       ///< Set PDCA channel reload values with address where data to load are stored, and size of the data block to load.
       pdca_reload_channel(PDCA_CHANNEL_DAC, (int8_t  *)buffer + 2 * from, to - from);
	}	   	
}


void init_pevc(void)
{
	///< PEVC Event Shaper options.
	static const pevc_evs_opt_t PEVC_EVS_OPTIONS = 
	{
	.igfdr = 0x0A,            ///< Set the IGF clock (don't care here).
	.igf = PEVC_EVS_IGF_OFF,  ///< Input Glitch Filter off
	.evf = PEVC_EVS_EVF_OFF,  ///< Enable Event on falling edge
	.evr = PEVC_EVS_EVR_ON    ///< Enable Event on rising edge
	};
  
	///< PEVC Init.
	///< Configuring the PEVC path: input is the generic clock, each clock cycle, the PEVC trigger a new DAC sample
	///< The sinus samples are sent through the PDCA.  
	///< a change on PEVC input pin0 event will trigger the PDCA channel 0/1 transfer
	pevc_channel_configure(ppevc,   AVR32_PEVC_ID_USER_DACIFB0_CHA, 
									AVR32_PEVC_ID_GEN_GCLK_0, 
									&PEVC_EVS_OPTIONS);							  
	///< Enable the PEVC channel 0.
	pevc_channels_enable(ppevc, 1 << AVR32_PEVC_ID_USER_DACIFB0_CHA);
}

void init_gclk(void)
{
  ///< Setup Sampling Rate
 
  ///< Setup the generic clock for EVENT
  scif_gc_setup(AVR32_SCIF_GCLK_GCLK2_EVENT, 
                SCIF_GCCTRL_SLOWCLOCK, 
                AVR32_SCIF_GC_DIV_CLOCK, 
                DAC_SAMPLE_CLOCK_DIVIDER);  
  ///< Now enable the generic clock
  scif_gc_enable(AVR32_SCIF_GCLK_GCLK2_EVENT);
}

void dac_dma_init(int32_t trigger_mode) {
	///< GPIO pin/dac-function map.
	static const gpio_map_t DACIFB_GPIO_MAP =
	{
		{AVR32_DACREF_PIN,AVR32_DACREF_FUNCTION},
		{AVR32_ADCREFP_PIN,AVR32_ADCREFP_FUNCTION},
		{AVR32_ADCREFN_PIN,AVR32_ADCREFN_FUNCTION},
		{DAC_AUDIO_PIN, DAC_AUDIO_FUNCTION}
	};
	///< DACIFB Configuration
	dacifb_opt_t dacifb_opt = 
	{
		.reference                  = DACIFB_REFERENCE_VDDANA,      ///< VDDANA Reference
		.channel_selection          = DAC_AUDIO_CHANNEL,			///< Selection Channels A&B
		.low_power                  = false,                        ///< Low Power Mode     
		.dual                       = false,                        ///< Dual Mode
		.prescaler_clock_hz         = DAC_PRESCALER_CLOCK,          ///< Prescaler Clock (Should be 500Khz)             
		.offset_calibration_value   = 0,
		.gain_calibration_value     = 1                 
	};
	///< DACIFB Channel Configuration
	dacifb_channel_opt_t dacifb_channel_opt = 
	{
		.auto_refresh_mode    = true,                ///< Auto Refresh Mode 
		.trigger_mode         = trigger_mode,        ///< Trigger selection
		.left_adjustment      = false,               ///< Right Adjustment
		.data_shift           = 0,                   ///< Number of Data Shift 
		.data_round_enable    = false                ///< Data Rouding Mode                                              };
	};
	///< Assign and enable GPIO pins to the DAC function.
	gpio_enable_module(DACIFB_GPIO_MAP, sizeof(DACIFB_GPIO_MAP) / sizeof(DACIFB_GPIO_MAP[0]));

	///< Get DACIFB Factory Configuration
	//dacifb_get_calibration_data(dacifb, &dacifb_opt, DAC_AUDIO_INSTANCE);
                              
	///< configure DACIFB
	if (dacifb_configure(dacifb, &dacifb_opt, FOSC0) == 0) 
	{
		print_util_dbg_print("error configuring DAC\r\n");
		return;
	}
	
	///< Enable and configure channel DACIFB
	if (dacifb_configure_channel(dacifb, dac_channel_audio, &dacifb_channel_opt, DAC_PRESCALER_CLOCK) ==0) 
	{
		print_util_dbg_print("error configuring DAC channel\r\n");
		return;
	}
  
	dacifb_start_channel(dacifb,
						dac_channel_audio,
						FOSC0);
}

void dac_dma_load_buffer(uint16_t* samples, int32_t from_sample, int32_t to_sample, int32_t repeat) 
{
	///< PDCA channel options
	buffer = samples;
	from = from_sample;
	to = to_sample;
	
	static  pdca_channel_options_t PDCA_OPTIONS =
	{
		.addr	= 0,									///< memory address
		.pid	= AVR32_PDCA_PID_DAC_TX,				///< select peripheral - data are transmit on the DAC output
		.size	= 0,									///< transfer counter
		.r_addr = NULL,									///< next memory address
		.r_size = 0,									///< next transfer counter
		.transfer_size = PDCA_TRANSFER_SIZE_HALF_WORD	///< select size of the transfer      
	};
	
	PDCA_OPTIONS.addr = (int8_t  *)samples + 2 * from;
	PDCA_OPTIONS.size = to - from;

	///< Initialize Event Controller
	init_pevc();
	///< Initialize Generic Clock
	init_gclk();

	///< Init PDCA channel with the pdca_options.
	pdca_init_channel(PDCA_CHANNEL_DAC, &PDCA_OPTIONS); ///< init PDCA channel with options.

	autoplay = repeat;
	//if (repeat==1) 
	//{
		// TODO: for some reason having this interrupt tends to crash occasionally
		INTC_register_interrupt( (__int_handler) &pdca_int_handler_dac, AVR32_PDCA_IRQ_4, AVR32_INTC_INT0);
		pdca_enable_interrupt_transfer_complete(PDCA_CHANNEL_DAC);
	//}  
} 

void dac_dma_play() 
{
	//Disable_global_interrupt();
	pdca_disable(PDCA_CHANNEL_DAC);
	pdca_reload_channel(PDCA_CHANNEL_DAC, (int8_t  *)buffer + 2 * from, to - from);
	///< Enable now the transfer.
	//Enable_global_interrupt();
	pdca_enable(PDCA_CHANNEL_DAC); 
}

void DAC_pause(void) {};

void DAC_resume(void) {};

int32_t  DAC_is_finished(void) 
{
	return 0;
}

void dac_dma_set_value(int32_t output) 
{
	dacifb->dr0 = (output); 
}



