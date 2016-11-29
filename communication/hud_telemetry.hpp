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
 * \file hud_telemetry.hpp
 *
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *
 * \brief This file sends the MAVLink HUD message
 *
 ******************************************************************************/


#ifndef HUD_TELEMETRY_HPP__
#define HUD_TELEMETRY_HPP__

#include "sensing/ins.hpp"
#include "sensing/ahrs.hpp"
#include "control/control_command.hpp"

/**
 * \brief   The HUD structure to send the MAVLink HUD message
 */
typedef struct
{
    const INS* ins;                         ///< The pointer to the Inertial Navigation System
    const command_t* controls;              ///< The pointer to the control structure
    const AHRS* ahrs;                       ///< The pointer to the attitude estimation structure
    const Mavlink_stream* mavlink_stream;   ///< The pointer to the MAVLink stream structure
} hud_telemetry_t;

/**
 * \brief   Initialise the HUD structure
 *
 * \param   hud          The pointer to the HUD structure
 * \param   ins          The pointer to the Inertial Navigation System
 * \param   controls     The pointer to the controls structure
 * \param   ahrs         The pointer to the attitude estimation structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool hud_telemetry_init(hud_telemetry_t* hud_telemetry_structure, const INS* ins, const command_t* controls, const AHRS* ahrs);

/**
 * \brief   Function to send the MAVLink HUD message
 *
 * \param   hud                 The pointer to the HUD structure
 * \param   mavlink_stream      The pointer to the MAVLink stream structure
 * \param   msg                 The pointer to the MAVLink message
 */
void hud_telemetry_send_message(const hud_telemetry_t* hud, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg);


#endif //HUD_TELEMETRY_HPP__
