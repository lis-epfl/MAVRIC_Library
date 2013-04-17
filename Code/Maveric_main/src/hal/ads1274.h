/** Driver for the external ADC: ADS1274  (4ch, 24 bit, SPI interface)
Author: Felix Schill  

All rights reserved (2011).
*/



#ifndef ADS1274_H
#define ADS1274_H
#include <avr32/io.h>
#include "preprocessor.h"
#include "compiler.h"
#include "user_board.h"


#define ADC_BUFFER_SIZE 1024
#define ADC_INPUT_CHANNELS 4
// -----  Pin configurations -------
#define ADC_SPI_PORT AVR32_SPI0
#define ADC_SPI_INDEX 0

// function pointer to a generator function that is called every time a sample is collected, to generate an output at the DAC pin
typedef uint16_t (*generatorfunction)(uint32_t);

/** set pointer to a callback function that creates a waveform for the DAC. 
 * input:  sample index (0 <= index <= ADC_BUFFER_SIZE)
 * output: a 12-bit value for the DAC to convert (0<= output < 4096)
 */
void set_DAC_generator_function(generatorfunction new_function_generator );

// Initializes ADC (configures Pins, starts Clock, sets defaults)
void Init_ADC(void);

// Enable/Disable the clock to the ADC
void ADC_Switch_Clock(Bool on_off);

// Switch the four input channels on or off
void ADC_Switch_Channel(int channel, Bool on_off);

// configures the ADC mode (refer to datasheet for options)
void ADC_Set_Mode(int mode);

// enables continuous sampling  -- not implemented yet
void ADC_Start_Sampling(void);

// starts sampling, captures one buffer length and then stops
void ADC_Start_Oneshot(void);

// stops sampling immediately
void ADC_Stop_Sampling(void);

// Returns true if one-shot sampling has finished
Bool Sampling_Complete(void);

int get_interrupt_counter(void);

int get_sampling_status(void);

float get_sample(int channel, int sample);
#endif
