/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file dac_dma.h
*
* This file is the driver for the Internal DAC (Digital to Analog Conversion) using the DMA
*/


#ifndef DAC_DMA_H
#define DAC_DMA_H
#include <avr32/io.h>
#include "preprocessor.h"
#include "compiler.h"
#include "user_board.h"
#include "dacifb.h"

#define DAC_MODE_DMA    DACIFB_TRIGGER_MODE_EVENT						///< Define the DAC mode in respect to the DMA usage
#define DAC_MODE_MANUAL DACIFB_TRIGGER_MODE_MANUAL						///< Define The DAC mode in manual usage

 // Sampling Rate = (115200/DAC_SAMPLING_CLOCK_DIVIDER) = 11'520Hz
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
void Init_DAC(int trigger_mode);

/**
 * \brief get the DAC sampling buffer
 *
 * \param samples pointer to the sampling buffer of the DAC
 * \param from_sample (?)
 * \param to_sample (?)
 * \param repeat (?)
*/
void DAC_load_buffer(uint16_t* samples, int from_sample, int to_sample, int repeat);

/**
 * \brief Play a sound using the DAC
*/
void DAC_play(void);

/*  // not implemented yet
void DAC_pause();

void DAC_resume();

int  DAC_is_finished();
*/

/**
 * \brief set a voltage on a pin using the DAC
 *
 * \param output desired voltage
*/
void DAC_set_value(int32_t output);

#endif