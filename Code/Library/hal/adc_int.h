
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

// Initializes ADC (configures Pins, starts Clock, sets defaults)
void Init_ADCI(uint32_t adc_frequency, uint8_t reference_source);

void clear_adc_sequencer(void);

int8_t adc_sequencer_add(int16_t *buffer, uint8_t input_p, uint8_t input_n, uint8_t gain);

// starts sampling, captures one buffer length and then stops
void ADCI_Start_Sampling(int length, int samplingrate, int set_oversampling, int set_oversampling_divider, bool continuous);

// stops sampling immediately
void ADCI_Stop_Sampling(void);

// Returns true if one-shot sampling has finished
Bool ADCI_Sampling_Complete(void);

int16_t ADCI_get_sample(int channel, int sample);

int16_t* ADCI_get_buffer(void);

int ADCI_get_sampling_status(void);

uint32_t get_adc_int_period(void);

#endif