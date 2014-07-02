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
* \file ads1274.h
*
* This file is the Analog to Digital Converter module using interruptions
*/


#ifndef ADC_INT_H
#define ADC_INT_H

#include "preprocessor.h"
#include "compiler.h"
#include "user_board.h"
#include "adcifa.h"


#define ADC_FREQUENCY 1000000
//#define OVERSAMPLING 8				// maximum sampling frequency: 1000000 / 4ch /2(HW-OVRSMP) /4 (OVERSAMPLING) = 31250 Hz
//#define OVERSAMPLING_DIVIDER 2

#define SLOTS_PER_SEQUENCER 8
#define MAX_CHANNELS 16

/**
 * \brief Initializes ADC (configures Pins, starts Clock, sets defaults)
 *
 * \param adc_frequency frequency of the ADC
 * \param reference_source voltage reference for the ADC
*/
void Init_ADCI(uint32_t adc_frequency, uint8_t reference_source);

/**
 * \brief Clear the ADC sequencer
*/
void clear_adc_sequencer(void);

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
int8_t adc_sequencer_add(int16_t *buffer, uint8_t input_p, uint8_t input_n, uint8_t gain);

/**
 * \brief Start the sampling of the ADC, captures one buffer length and then stops
 *
 * \param length number of samples
 * \param samplingrate sampling rate
 * \param set_oversampling oversampling
 * \param set_oversampling_divider oversampling divider
 * \param continuous define whether we will mode in continuous or discrete mode
*/
void ADCI_Start_Sampling(int length, int samplingrate, int set_oversampling, int set_oversampling_divider, bool continuous);

/**
 * \brief stops sampling immediately
*/
void ADCI_Stop_Sampling(void);

/**
 * \brief get whether one-shot sampling has finished
 * 
 * \return Returns true if one-shot sampling has finished
*/
Bool ADCI_Sampling_Complete(void);

/**
 * \brief return a specific sample of the ADC channel
 *
 * \param channel which channel of the ADC
 * \param sample which sample we are interested in
 *
 * \return that specific sample for the given channel
*/
int16_t ADCI_get_sample(int channel, int sample);

/**
 * \brief return a 2 dimension table of samples for all channels
 *
 * \return a 2 dimension table of samples for all channels
*/
int16_t* ADCI_get_buffer(void);

/**
 * \brief return the sampling counter, which indicates where the sampling process stand
 *
 * \return the sampling counter
*/
int ADCI_get_sampling_status(void);

/**
 * \brief return the period of the ADC interruption
 *
 * \return the period of the ADC interruption
*/
uint32_t get_adc_int_period(void);

#endif