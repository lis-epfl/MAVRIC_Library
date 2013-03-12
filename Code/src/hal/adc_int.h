
#ifndef ADC_INT_H
#define ADC_INT_H
#include <avr32/io.h>
#include "preprocessor.h"
#include "compiler.h"
#include "user_board.h"


#define ADCI_BUFFER_SIZE 512
#define ADCI_INPUT_CHANNELS 4

int32_t adci_buffer[ADCI_BUFFER_SIZE][ADCI_INPUT_CHANNELS];

// Initializes ADC (configures Pins, starts Clock, sets defaults)
void Init_ADCI();

// enables continuous sampling  -- not implemented yet
void ADCI_Start_Sampling();

// starts sampling, captures one buffer length and then stops
void ADCI_Start_Oneshot();

// stops sampling immediately
void ADCI_Stop_Sampling();

// Returns true if one-shot sampling has finished
Bool ADCI_Sampling_Complete();


#endif