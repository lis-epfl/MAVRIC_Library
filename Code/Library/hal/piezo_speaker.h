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

#define PIEZO_HIGH_PIN AVR32_PIN_PA12			///< Define the Microcontroller pin associated with the high pin of the piezo speaker
#define PIEZO_LOW_PIN AVR32_PIN_PA15			///< Define the Microcontroller pin associated with the low pin of the piezo speaker

/**
 * \brief Initialize the piezo speaker
 */
void init_piezo_speaker();

/**
 * \brief Initialize the speaker in a binary(?) mode
 */
void init_piezo_speaker_binary();

/**
 * \brief instantaneous output voltage sent to the speaker 
 * to make sounds this needs to be called repeatedly.
 *
 * \param analog_value sent to speaker
 */
void set_value(int analog_value);

/**
 * \brief Set a beeping 
 *
 * \param  binary_value different beep depending on the sign of binary_value 
 */
void set_value_binary(int binary_value);

/**
 * \brief Beep at a given frequency for a duration
 *
 * \param duration_ms Duration of the beep
 * \param frequency Frequency of the beep
 */
void beep(int duration_ms, int frequency);


#endif /* PIEZO_SPEAKER_H_ */