/*
 * piezo_speaker.h
 *
 * Created: 27/03/2014 18:20:04
 *  Author: schill
 */ 


#ifndef PIEZO_SPEAKER_H_
#define PIEZO_SPEAKER_H_

#include <stdint.h>

//#define PIEZO_HIGH_PIN AVR32_PIN_PA12
//#define PIEZO_LOW_PIN AVR32_PIN_PA15

void piezo_speaker_init();
void piezo_speaker_init_binary();

// instantaneous output voltage sent to the speaker - to make sounds this needs to be called repeatedly.
void piezo_speaker_set_value(int32_t analog_value);
void piezo_speaker_set_value_binary(int32_t binary_value);

void piezo_speaker_beep(int32_t duration_ms, int32_t frequency);


#endif /* PIEZO_SPEAKER_H_ */