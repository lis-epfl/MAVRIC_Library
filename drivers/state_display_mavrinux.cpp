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
 * \file state_display_mavrinux.cpp
 *
 * \author MAV'RIC Team
 * \author Jean-François Burnier
 *
 * \brief Interface class for state display for linux sim
 *
 ******************************************************************************/

 #include "drivers/state_display_mavrinux.hpp"
 #include "util/print_util.h"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

State_display_mavrinux::State_display_mavrinux()
{
	state_ 	   	= MAV_STATE_CALIBRATING;
	state_old_ 	= MAV_STATE_ACTIVE;
	state_ptr_ 	= NULL;
	idle_ 		= 0;
}

bool State_display_mavrinux::update(void)
{
	bool init;

	state_ = *state_ptr_;
	init = state_old_ != state_;

	if (init)
	{
		print_util_dbg_print("Mav entering new state:\t");

		switch (state_)
	 	{
	 		case MAV_STATE_CALIBRATING:
	 			print_util_dbg_print("Calibrating\n");
	 			break;

	 		case MAV_STATE_STANDBY:
	 			print_util_dbg_print("Standby\n");
	 			break;

	 		case MAV_STATE_ACTIVE:
	 			print_util_dbg_print("Active\n");
	 			break;

	 		case MAV_STATE_CRITICAL:
	 			print_util_dbg_print("Critical\n");
	 			break;

	 		case MAV_STATE_EMERGENCY:
	 			print_util_dbg_print("Emergency\n");

	 		default:
	 			print_util_dbg_print("Unknow State\n");
	 			break;
	 	}
 	}

 	state_old_ = state_;
 	return true;
}