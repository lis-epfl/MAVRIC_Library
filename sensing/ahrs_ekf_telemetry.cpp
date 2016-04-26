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
 * \file ahrs_ekf_telemetry.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of the telemetery for the EKF attitude estimation
 *
 ******************************************************************************/

#include "sensing/ahrs_ekf_telemetry.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
#include "util/constants.h"
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Function to start/stop the north vector calibration
 *
 * \param   ahrs_ekf                The pointer to the EKF attitude estimation structure
 * \param   packet                  The pointer to the MAVLink command long structure
 *
 * \return  The MAV_RESULT of the command
 */
static mav_result_t ahrs_ekf_telemetry_calibrate_north_vector(Ahrs_ekf* ahrs_ekf, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t ahrs_ekf_telemetry_calibrate_north_vector(Ahrs_ekf* ahrs_ekf, mavlink_command_long_t* packet)
{
	mav_result_t result;

	if (packet->param1 == 2)
	{
		result = MAV_RESULT_ACCEPTED;

		if (!ahrs_ekf->calibrating_north_vector_)
		{
			ahrs_ekf->calibrating_north_vector_ = true;

			print_util_dbg_print("Starting North vector calibration\r\n");
			print_util_dbg_print("Old North vector :");
			print_util_dbg_print_vector(ahrs_ekf->mag_global_,5);
			print_util_dbg_print("\r\n");

			for (uint16_t i = 0; i < 3; ++i)
			{
				ahrs_ekf->mag_lpf_[i] = ahrs_ekf->imu_.mag()[i];
			}
			
		}
		else
		{
			ahrs_ekf->calibrating_north_vector_ = false;
		}
		
	}
	else
	{
		result = MAV_RESULT_UNSUPPORTED;
	}

	return result;

}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool ahrs_ekf_telemetry_init(const Ahrs_ekf* ahrs_ekf, Mavlink_message_handler* message_handler)
{
	bool init_success = true;

    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS; // 242
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)            &ahrs_ekf_telemetry_calibrate_north_vector;
    callbackcmd.module_struct  = (Mavlink_message_handler::handling_module_struct_t) ahrs_ekf;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);

    return init_success;

}