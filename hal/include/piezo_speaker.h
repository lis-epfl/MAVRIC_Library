/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file piezo_speaker.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for the piezzo speaker
 *
 ******************************************************************************/


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
 * \param	duration_ms		Duration of the piezo_speaker_beep
 * \param	frequency		Frequency of the piezo_speaker_beep
 */
void piezo_speaker_beep(int32_t duration_ms, int32_t frequency);


/**
 * \brief Startup melody
 */
void piezo_speaker_startup_melody(void);

/**
 * \brief Critical error melody
 */
void piezo_speaker_critical_error_melody(void);


/**
 * \brief Quick startup melody
 */
void piezo_speaker_quick_startup(void);

/**
 * \brief Quick startup melody
 */
void piezo_speaker_startup_bb(void);

/**
 * \brief Star wars melody
 */
void piezo_speaker_star_wars(void);


/**
 * \brief Mario melody
 */
void piezo_speaker_mario_melody(void);



#ifdef __cplusplus
	}
#endif

#endif /* PIEZO_SPEAKER_H_ */