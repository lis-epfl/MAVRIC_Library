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
 * \file dac_dma.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for the Internal DAC 
 * (Digital to Analog Conversion) using the DMA
 * 
 ******************************************************************************/


#ifndef DAC_DMA_H
#define DAC_DMA_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <avr32/io.h>
#include "preprocessor.h"
#include <stdint.h>
#include "user_board.h"
#include "dacifb.h"

#define DAC_MODE_DMA    DACIFB_TRIGGER_MODE_EVENT						///< Define the DAC mode in respect to the DMA usage
#define DAC_MODE_MANUAL DACIFB_TRIGGER_MODE_MANUAL						///< Define The DAC mode in manual usage

 ///< Sampling Rate = (115200 / DAC_SAMPLING_CLOCK_DIVIDER) = 11'520Hz
#define DAC_SAMPLE_CLOCK_DIVIDER 10										///< Define the Clock Divider for the sampling of the DAC

#  define DAC_AUDIO_INSTANCE            0								///< Define the audio instance for the DAC
#  define DAC_AUDIO_CHANNEL             DACIFB_CHANNEL_SELECTION_A		///< Define the audio channel for the DAC
#  define DAC_AUDIO_PIN                 AVR32_DAC1A_PIN					///< Define the audio pin used for the DAC
#  define DAC_AUDIO_FUNCTION            AVR32_DAC1A_PIN					///< Associate the audio function with the pin used for the DAC 
#  define PDCA_CHANNEL_DAC              4								///< Define the channel number for the DAC
#  define AVR32_PDCA_PID_DAC_TX         AVR32_PDCA_PID_DACIFB0_CHA_TX	///< Select peripheral - data are transmit on the DAC output
#  define DAC_PRESCALER_CLOCK           5000000							///< Prescaler Clock of the DAC

/**
 * \brief Initialize the DAC module
 *
 * \param trigger_mode trigger selection for the DAC mode
*/
void dac_dma_init(int32_t trigger_mode);

/**
 * \brief get the DAC sampling buffer
 *
 * \param samples pointer to the sampling buffer of the DAC
 * \param from_sample (?)
 * \param to_sample (?)
 * \param repeat (?)
*/
void dac_dma_load_buffer(uint16_t* samples, int32_t from_sample, int32_t to_sample, int32_t repeat);

/**
 * \brief Play a sound using the DAC
*/
void dac_dma_play(void);

///< not implemented yet
/*
void DAC_pause();
void DAC_resume();
int32_t  DAC_is_finished();
*/

/**
 * \brief set a voltage on a pin using the DAC
 *
 * \param output desired voltage
*/
void dac_dma_set_value(int32_t output);

#ifdef __cplusplus
	}
#endif

#endif