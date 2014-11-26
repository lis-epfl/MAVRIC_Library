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
 * \file remote_default_config.h
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief Default configuration for the remote module
 *
 ******************************************************************************/


#ifndef REMOTE_DEFAULT_CONFIG_H_
#define REMOTE_DEFAULT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "remote.h"


remote_conf_t remote_default_config =
{
	.type = REMOTE_TURNIGY,
	.mode_config =
	{
		.safety_channel = CHANNEL_GEAR,
		.safety_mode =
		{
			.byte = MAV_MODE_ATTITUDE_CONTROL,
			// .flags =
			// {
			// .MANUAL = MANUAL_ON,
			// }
		},
		.mode_switch_channel = CHANNEL_FLAPS,
		.mode_switch_up =
		{
			.byte = MAV_MODE_VELOCITY_CONTROL
			// .flags =
			// {
			// .MANUAL = MANUAL_ON,
			// .STABILISE = STABILISE_ON,
			// }
		},
		.mode_switch_middle =
		{
			.byte = MAV_MODE_POSITION_HOLD,
			// .flags =
			// {
			// .MANUAL = MANUAL_ON,
			// .GUIDED = GUIDED_ON,
			// }
		},
		.mode_switch_down =
		{
			.byte = MAV_MODE_GPS_NAVIGATION
			// .flags =
			// {
			// .AUTO = AUTO_ON,
			// }
		},
		.use_custom_switch = true,//false,
		.custom_switch_channel = CHANNEL_AUX1,
		.use_test_switch = false,
		.test_switch_channel = CHANNEL_AUX2,
		.use_disable_remote_mode_switch = false,
		.test_switch_channel = CHANNEL_AUX2,
	},
};

#ifdef __cplusplus
}
#endif

#endif // REMOTE_DEFAULT_CONFIG_H_