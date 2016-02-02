/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file battery.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *   
 * \brief Battery voltage monitor
 *
 ******************************************************************************/
 
#include <stdint.h>

#include "drivers/battery.hpp"

extern "C"
{
	#include "util/print_util.h"
	#include "hal/common/time_keeper.hpp"
}
 

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
static float get_full_voltage(battery_type_t type);


/**
 * \brief	Returns the value of a low battery level
 *
 * \param	type		The battery type
 *
 * \return	The voltage of a low battery
 */
static float get_low_voltage(battery_type_t type);


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Battery::Battery(Adc& adc, battery_conf_t config):
	adc_( adc ),
	config_( config ),
	voltage_( get_full_voltage(config_.type) ),
	level_( 100.0f ),
	last_update_us_( time_keeper_get_us() ),
	is_low_( false )
{}


bool Battery::update(void)
{
	// Update ADC
	adc_.update();

    // Update voltage
    voltage_ =  config_.lpf_gain * voltage_ +
                (1.0f - config_.lpf_gain) * adc_.voltage();
    
    // Update current level
    level_ = 100.0f * ( voltage_ - get_low_voltage(config_.type) ) / 
			( get_full_voltage(config_.type) - get_low_voltage(config_.type) );


	// Update low level flag
	if( voltage_ < get_low_voltage(config_.type) )
	{
		is_low_ = true;
	}
	else
	{
		is_low_ = false;
	}

	// Save last update
    last_update_us_ = time_keeper_get_us();

    return true;
}


const float& Battery::voltage(void) const
{
	return voltage_;
}


const float& Battery::level(void) const
{
	return level_;
}



bool Battery::is_low(void) const
{
	return is_low_;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static float get_full_voltage(battery_type_t type)
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


static float get_low_voltage(battery_type_t type)
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
