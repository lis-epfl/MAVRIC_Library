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
 * \file state_display_megafly_rev4.cpp
 *
 * \author MAV'RIC Team
 * \author Jean-François Burnier
 *
 * \brief Interface class for state display for avr32
 *
 ******************************************************************************/

#include "drivers/state_display_megafly_rev4.hpp"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

State_display_megafly_rev4::State_display_megafly_rev4(Led& led_green, Led& led_red) : led_green_(led_green), led_red_(led_red)
{
	state_ 	   	= MAV_STATE_CALIBRATING;
	state_old_ 	= MAV_STATE_CALIBRATING;
	state_ptr_ 	= NULL;
	idle_ 		= 0;
}

bool State_display_megafly_rev4::update(void)
{
	bool init;
	state_ = *state_ptr_;
	init = state_old_ != state_;

	switch(state_)
 	{
 		case MAV_STATE_CALIBRATING:
 			if(init)
 			{
 				led_green_.on();
 				led_red_.off();
 				idle_ = 0;
 			}
 			else if(idle_ >= 6)
 			{
 				led_green_.toggle();
 				led_red_.toggle();
 				idle_ = 0;
 			}
 			else
 				idle_++;
 			break;

 		case MAV_STATE_STANDBY:
 			if(init)
 			{
 				led_red_.off();
 				idle_ = 0;
 			}
 			else if(idle_ >= 6)
 			{
 				led_green_.toggle();
 				idle_ = 0;
 			}
 			else
 				idle_++;

 			break;

 		case MAV_STATE_ACTIVE:
 			if(init)
 			{
 				led_red_.off();
 				idle_ = 0;
 			}
 			else if(idle_ >= 2)
 			{
 				led_green_.toggle();
 				idle_ = 0;
 			}
 			else
 				idle_++;

 			break;

 		case MAV_STATE_CRITICAL:
 			if(init)
 			{
 				led_red_.on();
 				idle_ = 0;
 			}
 			else if(idle_ >= 2)
 			{
 				led_green_.toggle();
 				idle_ = 0;
 			}
 			else
 				idle_++;

 			break;

 		case MAV_STATE_EMERGENCY:
 			led_red_.toggle();
 			if(idle_ >= 2)
 			{
 				led_green_.toggle();
 				idle_ = 0;
 			}
 			else
 				idle_++;
 			break;

 		default:
 			break;
 	}

 	state_old_ = state_;

 	return true;
}
