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
* \file adc_int.h
*
* This file is the Analog to Digital Converter module using interruptions
*/


#ifndef ADC_INT_H
#define ADC_INT_H

#ifdef __cplusplus
	extern "C" {
#endif

#include "preprocessor.h"
#include <stdint.h>
#include <stdbool.h>
#include "user_board.h"
#include "adcifa.h"


#define ADC_FREQUENCY 1000000			///< Define the frequency of the ADC
//#define OVERSAMPLING 8				///< maximum sampling frequency: 1000000 / 4ch /2(HW-OVRSMP) /4 (OVERSAMPLING) = 31250 Hz
//#define OVERSAMPLING_DIVIDER 2

#define SLOTS_PER_SEQUENCER 8			///< Define the number of slots per sequencer
#define MAX_CHANNELS 16					///< Define the maximum number of channels

/**
 * \brief Initializes ADC (configures Pins, starts Clock, sets defaults)
 *
 * \param adc_frequency frequency of the ADC
 * \param reference_source voltage reference for the ADC
*/
void adc_int_init(uint32_t adc_frequency, uint8_t reference_source);

/**
 * \brief Clear the ADC sequencer
*/
void adc_int_clear_sequencer(void);

/**
 * \brief Add an ADC sequencer, 
 *
 * \param buffer a pointer to the buffer in which you will store the ADC samples
 * \param input_p the positive input: either AVR32_ADCIFA_INP_GNDANA, either the specific pin eg: AVR32_ADCIFA_INP_ADCIN6
 * \param input_n the negative input: either AVR32_ADCIFA_INN_GNDANA, either the specific pin eg: AVR32_ADCIFA_INN_ADCIN11
 * \param gain gain conversion factor for the ADC, eg: ADCIFA_SHG_1
 *
 * \return return -1 if you have already reach the max number of sequencer, otherwise return the sequence number
*/
int8_t adc_int_sequencer_add(int16_t *buffer, uint8_t input_p, uint8_t input_n, uint8_t gain);

/**
 * \brief Start the sampling of the ADC, captures one buffer length and then stops
 *
 * \param length number of samples
 * \param samplingrate sampling rate
 * \param set_oversampling oversampling
 * \param set_oversampling_divider oversampling divider
 * \param continuous define whether we will mode in continuous or discrete mode
*/
void adc_int_start_sampling(int32_t length, int32_t samplingrate, int32_t set_oversampling, int32_t set_oversampling_divider, bool continuous);

/**
 * \brief stops sampling immediately
*/
void adc_int_stop_sampling(void);

/**
 * \brief get whether one-shot sampling has finished
 * 
 * \return Returns true if one-shot sampling has finished
*/
bool adc_int_sampling_complete(void);

/**
 * \brief return a specific sample of the ADC channel
 *
 * \param channel which channel of the ADC
 * \param sample which sample we are interested in
 *
 * \return that specific sample for the given channel
*/
int16_t adc_int_get_sample(int32_t channel, int32_t sample);

/**
 * \brief return a 2 dimension table of samples for all channels
 *
 * \return a 2 dimension table of samples for all channels
*/
int16_t** adc_int_get_buffer(void);

/**
 * \brief return the sampling counter, which indicates where the sampling process stand
 *
 * \return the sampling counter
*/
int32_t adc_int_get_sampling_status(void);

/**
 * \brief return the period of the ADC interruption
 *
 * \return the period of the ADC interruption
*/
uint32_t adc_int_get_period(void);

#ifdef __cplusplus
	}
#endif

#endif