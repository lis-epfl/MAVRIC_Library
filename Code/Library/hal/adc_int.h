
#ifndef ADC_INT_H
#define ADC_INT_H
#include <avr32/io.h>
#include "preprocessor.h"
#include "compiler.h"
#include "user_board.h"


#define ADCI_BUFFER_SIZE 512
#define ADCI_INPUT_CHANNELS 4
#define ADC_FREQUENCY 1000000
#define OVERSAMPLING 8				// maximum sampling frequency: 1000000 / 4ch /2(HW-OVRSMP) /4 (OVERSAMPLING) = 31250 Hz
#define OVERSAMPLING_DIVIDER 2

#define RADAR_0_I_INP         AVR32_ADCIFA_INP_ADCIN0
#define RADAR_0_I_INN         AVR32_ADCIFA_INN_ADCIN8
#define RADAR_0_I_PIN         AVR32_ADCIN0_PIN
#define RADAR_0_I_FUNCTION    AVR32_ADCIN0_FUNCTION

#define RADAR_0_Q_INP         AVR32_ADCIFA_INP_ADCIN1
#define RADAR_0_Q_INN         AVR32_ADCIFA_INN_ADCIN8
#define RADAR_0_Q_PIN         AVR32_ADCIN2_PIN
#define RADAR_0_Q_FUNCTION    AVR32_ADCIN2_FUNCTION

#define RADAR_1_I_INP         AVR32_ADCIFA_INP_ADCIN3
#define RADAR_1_I_INN         AVR32_ADCIFA_INN_ADCIN8
#define RADAR_1_I_PIN         AVR32_ADCIN3_PIN
#define RADAR_1_I_FUNCTION    AVR32_ADCIN3_FUNCTION

#define RADAR_1_Q_INP         AVR32_ADCIFA_INP_ADCIN4
#define RADAR_1_Q_INN         AVR32_ADCIFA_INN_ADCIN8
#define RADAR_1_Q_PIN         AVR32_ADCIN4_PIN
#define RADAR_1_Q_FUNCTION    AVR32_ADCIN4_FUNCTION




// Initializes ADC (configures Pins, starts Clock, sets defaults)
void Init_ADCI();

// starts sampling, captures one buffer length and then stops
void ADCI_Start_Oneshot(int samplingrate);

// stops sampling immediately
void ADCI_Stop_Sampling();

// Returns true if one-shot sampling has finished
Bool ADCI_Sampling_Complete();

int16_t ADCI_get_sample(int channel,int even_odd, int sample);

int16_t* ADCI_get_buffer(int channel,int even_odd);

int ADCI_get_sampling_status();

#endif