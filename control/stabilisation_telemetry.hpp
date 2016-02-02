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
 * \file stabilisation_telemetry.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the stabilisation module
 *
 ******************************************************************************/

#ifndef STABILISATION_TELEMETRY_H_
#define STABILISATION_TELEMETRY_H_

#include "control/stabilisation_copter.hpp"
#include "communication/mavlink_stream.hpp"

extern "C" 
{
	#include "control/stabilisation.h"
}


/**
 * \brief	Function to send the MAVLink roll, pitch, yaw angular speeds and thrust setpoints message
 *
 * \param	stabiliser				The pointer to the structure containing the PID controller
 * \param	mavlink_stream			The pointer to the MAVLink stream structure
 * \param	msg						The pointer to the MAVLink message
 */
void stabilisation_telemetry_send_rpy_speed_thrust_setpoint(const stabiliser_t* stabiliser, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);

/**
 * \brief	Function to send the MAVLink roll, pitch and yaw errors message
 * 
 * \param	stabiliser	Pointer to the structure containing the PID controller
 * \param	mavlink_stream			The pointer to the MAVLink stream structure
 * \param	msg						The pointer to the MAVLink message
 */
void stabilisation_telemetry_send_rpy_rates_error(const stabiliser_t* stabiliser, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);

/**
 * \brief	Function to send the MAVLink roll, pitch, yaw and thrust setpoints message
 *
 * \param	controls				The pointer to the controls structure
 * \param	mavlink_stream			The pointer to the MAVLink stream structure
 * \param	msg						The pointer to the MAVLink message
 */
void stabilisation_telemetry_send_rpy_thrust_setpoint(const control_command_t* controls, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);

/**
 * \brief	Task to send the MAVLink command message depending on the control mode
 *
 * \param	controls				The pointer to the controls structure
 * \param	mavlink_stream			The pointer to the MAVLink stream structure
 * \param	msg						The pointer to the MAVLink message
 */
void stabilisation_telemetry_send_control(const control_command_t* controls, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);

/**
 * \brief	Task to send the MAVLink messages with the output of the PID controllers
 *
 * \param	stabilisation_copter	The pointer to the stabilisation copter structure
 * \param	mavlink_stream			The pointer to the MAVLink stream structure
 * \param	msg						The pointer to the MAVLink message
 */
void stabilisation_copter_send_outputs(stabilisation_copter_t* stabilisation_copter, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);


#endif /* STABILISATION_TELEMETRY_H_ */