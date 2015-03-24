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
 * \file battery.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *   
 * \brief Takes care of the battery level
 *
 ******************************************************************************/
 
#include "battery.h"
#include "print_util.h"
#include "time_keeper.h"
 
#include <stdint.h>


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Returns the battery level depending on the battery type
 *
 * \param	type		The battery type
 *
 * \return	The voltage of the full battery
 */
static float battery_get_full_voltage(battery_type_t type);

/**
 * \brief	Returns the value of a low battery level
 *
 * \param	type		The battery type
 *
 * \return	The voltage of a low battery
 */
static float battery_get_low_voltage(battery_type_t type);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static float battery_get_full_voltage(battery_type_t type)
{
	float full_voltage = 0.0f;
	
    switch(type)
    {
        case BATTERY_LIPO_1S:
			full_voltage =  4.2f;
			break;
			
        case BATTERY_LIPO_2S:
			full_voltage =  2 * 4.2f;
			break;
			
        case BATTERY_LIPO_3S:
			full_voltage =  3 * 4.2f;
			break;
			
        case BATTERY_LIPO_4S:
			full_voltage =  4 * 4.2f;
			break;
			
        case BATTERY_LIFE_1S:
			full_voltage =  3.6f;
			break;
			
        case BATTERY_LIFE_2S:
			full_voltage =  2 * 3.6f;
			break;
			
        case BATTERY_LIFE_3S:
			full_voltage =  3 * 3.6f;
			break;
			
        case BATTERY_LIFE_4S:
			full_voltage =  4 * 3.6f;
			break;
    }
	
	return full_voltage;
}


static float battery_get_low_voltage(battery_type_t type)
{
	float low_voltage = 0.0f;

    switch(type)
    {
        case BATTERY_LIPO_1S:
			low_voltage = 3.2f;
			break;
			
        case BATTERY_LIPO_2S:
			low_voltage = 2 * 3.2f;
			break;
			
        case BATTERY_LIPO_3S:
			low_voltage = 3 * 3.2f;
			break;
			
        case BATTERY_LIPO_4S:
			low_voltage = 4 * 3.2f;
			break;
			
        case BATTERY_LIFE_1S:
			low_voltage = 3.0f;
			break;
			
        case BATTERY_LIFE_2S:
			low_voltage = 2 * 3.0f;
			break;
			
        case BATTERY_LIFE_3S:
			low_voltage = 3 * 3.0f;
			break;
			
        case BATTERY_LIFE_4S:
			low_voltage = 4 * 3.0f;
			break;
    }
	
	return low_voltage;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool battery_init(battery_t* battery, battery_type_t type, float low_level_limit)
{
	bool init_success = true;
	
    battery->type				= type;
    battery->current_voltage	= battery_get_low_voltage(battery->type);
    battery->current_level		= 0.0f;
	battery->low_level_limit	= low_level_limit;
    battery->is_low				= false;
    battery->last_update_ms		= 0;
    battery->do_LPF				= true;
	battery->lpf_gain			= 0.99f;
	
	print_util_dbg_print("[BATTERY]: Initialized.\r\n");
	
	return init_success;
}


float battery_get_level(battery_t* battery)
{
    float level =   100.0f * ( battery->current_voltage - battery_get_low_voltage(battery->type) ) / 
                    ( battery_get_full_voltage(battery->type) - battery_get_low_voltage(battery->type) );

    return level;
}


void battery_update(battery_t* battery, float voltage)
{
    uint32_t now = time_keeper_get_millis();

    // Update voltage
    if(battery->do_LPF == true)
    {
        battery->current_voltage =  battery->lpf_gain * voltage +
                                    (1.0f - battery->lpf_gain) * battery->current_voltage;
    }
    else
    {
        battery->current_voltage =  voltage;
    }

    // Update current level
    battery->current_level = battery_get_level(battery);    


    // Check if low level
    if( battery->current_level < battery->low_level_limit )
    {
        battery->is_low = true;
    }
    else
    {
        battery->is_low = false;
    }

    // Save last update
    battery->last_update_ms = now;
}