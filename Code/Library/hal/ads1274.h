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
* This file is the Driver for the external ADC: ADS1274  (4ch, 24 bit, SPI interface)
*/



#ifndef ADS1274_H
#define ADS1274_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <avr32/io.h>
#include "preprocessor.h"
#include <stdint.h>
#include <stdbool.h>
#include "user_board.h"


#define ADC_BUFFER_SIZE 1024
#define ADC_INPUT_CHANNELS 4
// -----  Pin configurations -------
#define ADC_SPI_PORT AVR32_SPI0
#define ADC_SPI_INDEX 0

/** 
* \brief function pointer to a generator function that is called every time a sample is collected, to generate an output at the DAC pin
*/
typedef uint16_t (*generatorfunction)(uint32_t);

/** 
 * \brief set pointer to a callback function that creates a waveform for the DAC. 
 * input:  sample index (0 <= index <= ADC_BUFFER_SIZE)
 * output: a 12-bit value for the DAC to convert (0<= output < 4096)
 *
 * \param new_function_generator a function pointer to a generator function
 */
void ads1274_set_DAC_generator_function(generatorfunction new_function_generator );

/** 
 * \brief Initializes ADC (configures Pins, starts Clock, sets defaults)
*/
void ads1274_init_DAC(void);

/** 
 * \brief Enable/Disable the clock to the ADC
 *
 * \param on_off to enable or disable the ADC clock
*/
void ads1274_ADC_switch_clock(bool on_off);

/** 
 * \brief Switch the four input channels on or off
 *
 * \param channel select ADC channel
 * \param on_off enable or disable ADC of that channel
*/
void ads1274_ADC_switch_channel(int32_t channel, bool on_off);

/** 
 * \brief configures the ADC mode (refer to datasheet for options)
 *
 * \param mode define in which mode the ADC will be used
*/
void ads1274_ADC_set_mode(int32_t mode);

/** 
 * \brief enables continuous sampling  -- not implemented yet
*/
void ads1274_ADC_start_sampling(void);

/** 
 * \brief starts sampling, captures one buffer length and then stops
*/
void ads1274_ADC_start_oneshot(void);

/** 
 * \brief stops sampling immediately
*/
void ads1274_ADC_stop_sampling(void);

/** 
 * \brief get whether one-shot sampling has finished
 *
 * \return true if one-shot sampling has finished
*/
bool Sampling_Complete(void);

/**
 * \brief return the interrupt counter
 *
 * \return interrupt counter
*/
int32_t get_interrupt_counter(void);

/**
 * \brief return the status of the sampling process
 *
 * \return sampling counter
*/
int32_t get_sampling_status(void);

/**
 * \brief return an ADC sample
 *
 * \param channel ADC channel
 * \param sample the sample number
 *
 * \return the sample corresponding to this sample number on this ADC channel
*/
float get_sample(int32_t channel, int32_t sample);

#ifdef __cplusplus
	}
#endif

#endif
