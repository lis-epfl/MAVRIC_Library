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
 * \file imu_telemetry.cpp
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the IMU
 *
 ******************************************************************************/

#include "imu_telemetry.hpp"

extern "C"
{
	#include "time_keeper.hpp"
	#include "print_util.h"
	#include "constants.h"
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Function to start/stop the calibration
 * 
 * \param	imu						The pointer to the IMU structure
 * \param	packet					The pointer to the MAVLink command long structure
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t imu_telemetry_start_calibration(Imu* imu, mavlink_command_long_t* packet);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t imu_telemetry_start_calibration(Imu* imu, mavlink_command_long_t* packet)
{
	bool success = true;
	mav_result_t result = MAV_RESULT_UNSUPPORTED;
	
	/** Trigger calibration. 
	 * | Gyro calibration: 0: no, 1: yes
	 * | Magnetometer calibration: 0: no, 1: yes
	 * | Ground pressure: 0: no, 1: yes
	 * | Radio calibration: 0: no, 1: yes
	 * | Accelerometer calibration: 0: no, 1: yes
	 * | Compass/Motor interference calibration: 0: no, 1: yes
	 * | Empty
	 */
	
	// Gyroscope bias calibration
	if( packet->param1 == 1 )
	{
		success &= imu->start_gyroscope_bias_calibration();
		if( success )
		{
			print_util_dbg_print("[IMU CALIB] Start gyroscope calibration\r\n");
		}
		else
		{
			print_util_dbg_print("[IMU CALIB] [ERROR] Failed to start gyroscope calibration\r\n");
		}
	}
	else
	{
		if( imu->stop_gyroscope_bias_calibration() )
		{		
			print_util_dbg_print("[IMU CALIB] Stop gyroscope calibration\r\n");
		}
	}
	

	// Magnetometer bias calibration
	if (packet->param2 == 1)
	{
		success &= imu->start_magnetometer_bias_calibration();
		if( success )
		{
			print_util_dbg_print("[IMU CALIB] Start magnetometer calibration\r\n");
		}
		else
		{
			print_util_dbg_print("[IMU CALIB] [ERROR] Failed to start magnetometer calibration\r\n");
		}
	}
	else
	{
		if( imu->stop_magnetometer_bias_calibration() )
		{		
			print_util_dbg_print("[IMU CALIB] Stop magnetometer calibration\r\n");
		}
	}
	

	// Barometer calibration
	if (packet->param3 == 1)
	{
		success = false;
		print_util_dbg_print("[IMU CALIB] [ERROR] Barometer calibration unsupported\r\n");
	}


	// Radio calibration	
	if (packet->param4 == 1)
	{
		success = false;
		print_util_dbg_print("[IMU CALIB] [ERROR] Barometer calibration unsupported\r\n");
	}
	

	// Accelerometer bias calibration
	if (packet->param5 == 1)
	{
		success &= imu->start_accelerometer_bias_calibration();
		if( success )
		{
			print_util_dbg_print("[IMU CALIB] Start accelerometer calibration\r\n");
		}
		else
		{
			print_util_dbg_print("[IMU CALIB] [ERROR] Failed to start accelerometer calibration\r\n");
		}
	}
	else
	{
		if( imu->stop_accelerometer_bias_calibration() )
		{		
			print_util_dbg_print("[IMU CALIB] Stop accelerometer calibration\r\n");
		}
	}


	// Result code
	if( success )
	{
		result = MAV_RESULT_ACCEPTED;
	}
	else
	{
		result = MAV_RESULT_TEMPORARILY_REJECTED;
	}
	
	return result;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool imu_telemetry_init(Imu* imu, mavlink_message_handler_t* message_handler)
{
	bool init_success = true;
	
	// Add callbacks for waypoint handler commands requests
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	callbackcmd.command_id = MAV_CMD_PREFLIGHT_CALIBRATION; // 241
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&imu_telemetry_start_calibration;
	callbackcmd.module_struct =									imu;
	init_success &= mavlink_message_handler_add_cmd_callback(message_handler, &callbackcmd);
	
	return init_success;
}

void imu_telemetry_send_scaled(const Imu* imu, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{	
	std::array<float, 3> acc  = imu->acc();
	std::array<float, 3> gyro = imu->gyro();
	std::array<float, 3> mag  = imu->mag();

	mavlink_msg_scaled_imu_pack(mavlink_stream->sysid,
								mavlink_stream->compid,
								msg,
								time_keeper_get_ms(),
								1000 * acc[X],
								1000 * acc[Y],
								1000 * acc[Z],
								1000 * gyro[X],
								1000 * gyro[Y],
								1000 * gyro[Z],
								1000 * mag[X],
								1000 * mag[Y],
								1000 * mag[Z]);
}
