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
 * \file battery.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *   
 * \brief Takes care of the battery level
 *
 ******************************************************************************/


#ifndef BATTERY_H_
#define BATTERY_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <stdbool.h>
#include <stdint.h>

/**
 * \brief Defines the battery type (number of cells)
 */
typedef enum
{
    BATTERY_LIPO_1S,
    BATTERY_LIPO_2S,
    BATTERY_LIPO_3S,
    BATTERY_LIPO_4S,
    BATTERY_LIFE_1S,
    BATTERY_LIFE_2S,
    BATTERY_LIFE_3S,
    BATTERY_LIFE_4S,
} battery_type_t;

/**
 * \brief Defines the battery structure
 */
typedef struct
{
    battery_type_t  type;							///< The battery type
    float           current_voltage;				///< The current voltage of the battery in V
    float           current_level;					///< The current level of the battery in %
	float			low_level_limit;				///< The lower limit in %
    bool            is_low;							///< Flag to tell whether the battery is low
    uint32_t        last_update_ms;					///< The time of the last update in ms
    bool            do_LPF;							///< Flag to low-pass filter the battery input or not
	float			lpf_gain;						///< The value of the low-pass filter gain
} battery_t;

/**
 * \brief Initialize the battery module
 *
 * \param battery		Pointer to the battery structure
 * \param type			The type of battery (number of cells)
 * \param low_limit		Lower limit of the battery level that will trigger the low battery flag
 *
 * \return	True if the init succeed, false otherwise
 */
bool battery_init(battery_t* battery, battery_type_t type, float low_limit);

/**
 * \brief	Returns the level of the battery in percentage
 *
 * \param battery	Pointer to the battery structure
 *
 * \return	The level of the battery
 */
float battery_get_level(battery_t* battery);

/**
 * \brief	Updates the battery voltage level
 *
 * \param battery	Pointer to the battery structure
 * \param voltage	Update measured battery voltage
 */
void battery_update(battery_t* battery, float voltage);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_H_