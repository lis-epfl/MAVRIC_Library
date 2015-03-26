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
 * \file imu_telemetry.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the IMU
 *
 ******************************************************************************/


#ifndef IMU_TELEMETRY_H_
#define IMU_TELEMETRY_H_

#include "mavlink_stream.h"
#include "mavlink_message_handler.h"
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief	Initialise the imu telemetry module
 * 
 * \param	imu						The pointer to the IMU structure
 * \param	message_handler			The pointer to the message handler structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool imu_telemetry_init(imu_t* imu, mavlink_message_handler_t* message_handler);

/**
 * \brief	Function to send the MAVLink scaled IMU message
 * 
 * \param	imu						The pointer to the IMU structure
 * \param	mavlink_stream			The pointer to the MAVLink stream structure
 * \param	msg						The pointer to the MAVLink message
 */
void imu_telemetry_send_scaled(const imu_t* imu, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);


/**
 * \brief	Function to send the MAVLink raw IMU message
 * 
 * \param	imu						The pointer to the IMU structure
 * \param	mavlink_stream			The pointer to the MAVLink stream structure
 * \param	msg						The pointer to the MAVLink message
 */
void imu_telemetry_send_raw(const imu_t* imu, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);

/**
 * \brief	Function to send the MAVLink debug vector message with biaises
 * 
 * \param	imu						The pointer to the IMU structure
 * \param	mavlink_stream			The pointer to the MAVLink stream structure
 * \param	msg						The pointer to the MAVLink message
 */
void imu_telemetry_send_biais(const imu_t* imu, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);

#ifdef __cplusplus
}
#endif

#endif /* IMU_TELEMETRY_H_ */