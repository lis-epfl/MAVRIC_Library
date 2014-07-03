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
#include "compiler.h"
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
void set_DAC_generator_function(generatorfunction new_function_generator );

/** 
 * \brief Initializes ADC (configures Pins, starts Clock, sets defaults)
*/
void Init_ADC(void);

/** 
 * \brief Enable/Disable the clock to the ADC
 *
 * \param on_off to enable or disable the ADC clock
*/
void ADC_Switch_Clock(Bool on_off);

/** 
 * \brief Switch the four input channels on or off
 *
 * \param channel select ADC channel
 * \param on_off enable or disable ADC of that channel
*/
void ADC_Switch_Channel(int channel, Bool on_off);

/** 
 * \brief configures the ADC mode (refer to datasheet for options)
 *
 * \param mode define in which mode the ADC will be used
*/
void ADC_Set_Mode(int mode);

/** 
 * \brief enables continuous sampling  -- not implemented yet
*/
void ADC_Start_Sampling(void);

/** 
 * \brief starts sampling, captures one buffer length and then stops
*/
void ADC_Start_Oneshot(void);

/** 
 * \brief stops sampling immediately
*/
void ADC_Stop_Sampling(void);

/** 
 * \brief get whether one-shot sampling has finished
 *
 * \return true if one-shot sampling has finished
*/
Bool Sampling_Complete(void);

/**
 * \brief return the interrupt counter
 *
 * \return interrupt counter
*/
int get_interrupt_counter(void);

/**
 * \brief return the status of the sampling process
 *
 * \return sampling counter
*/
int get_sampling_status(void);

/**
 * \brief return an ADC sample
 *
 * \param channel ADC channel
 * \param sample the sample number
 *
 * \return the sample corresponding to this sample number on this ADC channel
*/
float get_sample(int channel, int sample);

#ifdef __cplusplus
	}
#endif

#endif
