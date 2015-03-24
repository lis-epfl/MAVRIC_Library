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
 * \file imu_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the IMU
 *
 ******************************************************************************/

#include "imu_telemetry.h"
#include "time_keeper.h"
#include "print_util.h"
#include "constants.h"

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
static mav_result_t imu_telemetry_start_calibration(imu_t* imu, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t imu_telemetry_start_calibration(imu_t* imu, mavlink_command_long_t* packet)
{
	mav_result_t result;
	int16_t i;

	print_util_dbg_print("Calibration cmd received");
	
	/* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Accelerometer calibration: 0: no, 1: yes| Compass/Motor interference calibration: 0: no, 1: yes| Empty|  */
	
	if ( (imu->state->mav_state == MAV_STATE_STANDBY)||(imu->state->mav_state == MAV_STATE_CALIBRATING) )
	{
		if  (packet->param1 == 1)
		{
			//if (!imu->calib_gyro.calibration)
			//{
			//print_util_dbg_print("Starting gyro calibration\r\n");
			//imu->calib_gyro.calibration = true;
			//imu->state->mav_state = MAV_STATE_CALIBRATING;
			//print_util_dbg_print("Old biais:");
			//print_util_dbg_print_vector(imu->calib_gyro.bias,2);
			//}
			//else
			//{
			//print_util_dbg_print("Stopping gyro calibration\r\n");
			//imu->calib_gyro.calibration = false;
			//imu->state->mav_state = MAV_STATE_STANDBY;
			//
			//for (i = 0; i < 3; i++)
			//{
			//imu->.bias[i] = (imu->calib_gyro.max_oriented_values[i] + imu->calib_gyro.min_oriented_values[i])/2.0f;
			//imu->calib_gyro.max_oriented_values[i] = -10000.0;
			//imu->calib_gyro.min_oriented_values[i] =  10000.0;
			//}
			//print_util_dbg_print("New biais:");
			//print_util_dbg_print_vector(imu->calib_gyro.bias,2);
			//}
			//result = MAV_RESULT_ACCEPTED;
			
			result = MAV_RESULT_UNSUPPORTED;
		}
		
		if (packet->param2 == 1)
		{
			if (!imu->calib_compass.calibration)
			{
				print_util_dbg_print("Starting magnetometers calibration\r\n");
				imu->calib_compass.calibration = true;
				imu->state->mav_state = MAV_STATE_CALIBRATING;
				print_util_dbg_print("Old biais:");
				print_util_dbg_print_vector(imu->calib_compass.bias,2);
			}
			else
			{
				print_util_dbg_print("Stopping compass calibration\r\n");
				imu->calib_compass.calibration = false;
				imu->state->mav_state = MAV_STATE_STANDBY;
				
				for (i = 0; i < 3; i++)
				{
					imu->calib_compass.bias[i] = (imu->calib_compass.max_oriented_values[i] + imu->calib_compass.min_oriented_values[i])/2.0f;
					imu->calib_compass.max_oriented_values[i] = -10000.0;
					imu->calib_compass.min_oriented_values[i] =  10000.0;
				}
				print_util_dbg_print("New biais:");
				print_util_dbg_print_vector(imu->calib_compass.bias,2);
			}
			result = MAV_RESULT_ACCEPTED;
		}
		
		if (packet->param3 == 1)
		{
			print_util_dbg_print("Starting ground pressure calibration\r\n");
			
			result = MAV_RESULT_UNSUPPORTED;
		}
		
		if (packet->param4 == 1)
		{
			print_util_dbg_print("Starting radio calibration\r\n");
			
			result = MAV_RESULT_UNSUPPORTED;
		}
		
		if (packet->param5 == 1)
		{
			//if (!imu->calib_accelero.calibration)
			//{
			//print_util_dbg_print("Starting accelerometers calibration\r\n");
			//imu->calib_accelero.calibration = true;
			//imu->state->mav_state = MAV_STATE_CALIBRATING;
			//print_util_dbg_print("Old biais:");
			//print_util_dbg_print_vector(imu->calib_accelero.bias,2);
			//}
			//else
			//{
			//print_util_dbg_print("Stopping accelerometer calibration\r\n");
			//imu->calib_accelero.calibration = false;
			//imu->state->mav_state = MAV_STATE_STANDBY;
			//
			//for (i = 0; i < 3; i++)
			//{
			//imu->calib_accelero.bias[i] = (imu->calib_accelero.max_oriented_values[i] + imu->calib_accelero.min_oriented_values[i])/2.0f;
			//imu->calib_accelero.max_oriented_values[i] = -10000.0;
			//imu->calib_accelero.min_oriented_values[i] =  10000.0;
			//}
			//print_util_dbg_print("New biais:");
			//print_util_dbg_print_vector(imu->calib_accelero.bias,2);
			//}
			//result = MAV_RESULT_ACCEPTED;
			
			result = MAV_RESULT_UNSUPPORTED;
		}
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

bool imu_telemetry_init(imu_t* imu, mavlink_message_handler_t* message_handler)
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

void imu_telemetry_send_scaled(const imu_t* imu, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{	
	mavlink_msg_scaled_imu_pack(mavlink_stream->sysid,
								mavlink_stream->compid,
								msg,
								time_keeper_get_millis(),
								1000 * imu->scaled_accelero.data[0],
								1000 * imu->scaled_accelero.data[1],
								1000 * imu->scaled_accelero.data[2],
								1000 * imu->scaled_gyro.data[0],
								1000 * imu->scaled_gyro.data[1],
								1000 * imu->scaled_gyro.data[2],
								1000 * imu->scaled_compass.data[0],
								1000 * imu->scaled_compass.data[1],
								1000 * imu->scaled_compass.data[2]);
}


void imu_telemetry_send_raw(const imu_t* imu, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_raw_imu_pack(	mavlink_stream->sysid,
								mavlink_stream->compid,
								msg,
								time_keeper_get_micros(),
								imu->oriented_accelero.data[0],
								imu->oriented_accelero.data[1],
								imu->oriented_accelero.data[2],
								imu->oriented_gyro.data[0],
								imu->oriented_gyro.data[1],
								imu->oriented_gyro.data[2],
								imu->oriented_compass.data[0],
								imu->oriented_compass.data[1],
								imu->oriented_compass.data[2]);
}

void imu_telemetry_send_biais(const imu_t* imu, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,"biaisGyro",
									time_keeper_get_micros(),
									imu->calib_gyro.bias[X],
									imu->calib_gyro.bias[Y],
									imu->calib_gyro.bias[Z]);
}
