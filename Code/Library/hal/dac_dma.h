/** Driver for the Internal DAC using DMA 
Author: Felix Schill  
All rights reserved (2011).
*/



#ifndef DAC_DMA_H
#define DAC_DMA_H
#include <avr32/io.h>
#include "preprocessor.h"
#include "compiler.h"
#include "user_board.h"
#include "dacifb.h"

#define DAC_MODE_DMA    DACIFB_TRIGGER_MODE_EVENT
#define DAC_MODE_MANUAL DACIFB_TRIGGER_MODE_MANUAL

 // Sampling Rate = (115200/DAC_SAMPLING_CLOCK_DIVIDER) = 11'520Hz
#define DAC_SAMPLE_CLOCK_DIVIDER 10

#  define DAC_AUDIO_INSTANCE            0
#  define DAC_AUDIO_CHANNEL             DACIFB_CHANNEL_SELECTION_A
#  define DAC_AUDIO_PIN                 AVR32_DAC0A_PIN
#  define DAC_AUDIO_FUNCTION            AVR32_DAC0A_PIN
#  define PDCA_CHANNEL_DAC              4
#  define AVR32_PDCA_PID_DAC_TX         AVR32_PDCA_PID_DACIFB0_CHA_TX
#  define DAC_PRESCALER_CLOCK           5000000



void Init_DAC(int trigger_mode);

void DAC_load_buffer(uint16_t* samples, int from_sample, int to_sample, int repeat);
void DAC_play(void);

/*  // not implemented yet
void DAC_pause();

void DAC_resume();

int  DAC_is_finished();
*/
void DAC_set_value(int32_t output);

#endif