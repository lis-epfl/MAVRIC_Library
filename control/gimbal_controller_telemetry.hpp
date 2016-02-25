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
 * \file gimbal_control_telemetry.hpp
 *
 * \author MAV'RIC Team
 * \author Alexandre Cherpillod
 *
 * \brief This module takes care of handling the received packet for the gimbal
 *
 ******************************************************************************/


#ifndef GIMBAL_CONTROL_TELEMETRY_H_
#define GIMBAL_CONTROL_TELEMETRY_H_

#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "control/gimbal_controller.hpp"


/**
 * \brief   Parse received MAVLink message in structure
 *
 * \param   gimbal	    The pointer to the gimbal structure
 * \param   sysid       The sysid of the system
 * \param   msg         The pointer to the MAVLink message received
 *
 * \return  The MAV_RESULT of the command
 */
void gimbal_telemetry_parse_msg(Gimbal_controller* gimbal_controller, uint32_t sysid, mavlink_message_t* msg);


/**
 * \brief   Initialise the manual_control telemetry module
 *
 * \param   gimbal_controller   The pointer to the gimbal_controller structure
 * \param   mavlink_stream      The pointer to the MAVLink stream structure
 * \param   message_handler     The pointer to the message handler
 *
 * \return  True if the init succeed, false otherwise
 */
bool gimbal_controller_telemetry_init(Gimbal_controller* gimbal_controller, mavlink_message_handler_t* message_handler);


#endif /* GIMBAL_CONTROL_TELEMETRY_H_ */
