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
* \file piezo_speaker.h
*
* This file is the driver for the piezzo speaker
*/


#ifndef PIEZO_SPEAKER_H_
#define PIEZO_SPEAKER_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

#define PIEZO_HIGH_PIN AVR32_PIN_PA12			///< Define the Microcontroller pin associated with the high pin of the piezo speaker
#define PIEZO_LOW_PIN AVR32_PIN_PA15			///< Define the Microcontroller pin associated with the low pin of the piezo speaker


/**
 * \brief Initialize the piezo speaker
 */
void piezo_speaker_init(void);


/**
 * \brief Initialize the speaker in a binary(?) mode
 */
void piezo_speaker_init_binary(void);


/**
 * \brief Beep at a given frequency for a duration
 *
 * \param duration_ms Duration of the piezo_speaker_beep
 * \param frequency Frequency of the piezo_speaker_beep
 */
void piezo_speaker_beep(int32_t duration_ms, int32_t frequency);


/**
 * @brief Startup melody
 */
void piezo_speaker_startup_melody(void);


void piezo_speaker_mario_melody(void);


#ifdef __cplusplus
	}
#endif

#endif /* PIEZO_SPEAKER_H_ */