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
 * \file manual_control_telemetry.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the state
 *
 ******************************************************************************/


#ifndef MANUAL_CONTROL_TELEMETRY_H_
#define MANUAL_CONTROL_TELEMETRY_H_

#include "mavlink_stream.hpp"
#include "mavlink_message_handler.hpp"
#include "manual_control.hpp"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief	Initialise the manual_control telemetry module
 *
 * \param	manual_control		The pointer to the manual_control structure
 * \param   mavlink_stream		The pointer to the MAVLink stream structure
 * \param	message_handler		The pointer to the message handler
 *
 * \return	True if the init succeed, false otherwise
 */
bool manual_control_telemetry_init(manual_control_t* manual_control, mavlink_message_handler_t *message_handler);


/**
 * \brief	Sends the manual control values via MAVLink
 * 
 * \param	manual_control			The pointer to the manual control structure
 * \param	mavlink_stream			The pointer to the MAVLink stream structure
 * \param	msg						The pointer to the MAVLink message
 */
void manual_control_telemetry_send(const manual_control_t* manual_control, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);


#ifdef __cplusplus
}
#endif

#endif /* MANUAL_CONTROL_TELEMETRY_H_ */