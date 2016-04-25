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
 * \file servos_telemetry.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the servos
 *
 ******************************************************************************/


#ifndef SERVOS_TELEMETRY_H_
#define SERVOS_TELEMETRY_H_

#include "communication/mavlink_stream.hpp"
#include "drivers/servo.hpp"

extern "C"
{
    // #include "servos.h"
}

/**
 * \brief The servo telemetry definition
 */
typedef struct
{
    Servo* servos[4];                   ///< The pointer to the first servo
} servos_telemetry_t;

/**
* \brief    Initializes the servos_telemetry_t structure by adding servos
*
* \param servos_telemetry           The pointer to the servos telemetry structure
* \param servo_0                    The zeroth servo
* \param servo_1                    The first servo
* \param servo_2                    The second servo
* \param servo_3                    The third servo
*/
void servos_telemetry_init(servos_telemetry_t* servos_telemetry, Servo* servo_0, Servo* servo_1, Servo* servo_2, Servo* servo_3);

/**
 * \brief   Sends the MAVLink message for the servos
 *
 * \param   servos_telemetry        The pointer to the servos_telemetry structure
 * \param   mavlink_stream          The pointer to the MAVLink stream structure
 * \param   msg                     The pointer to the MAVLink message
 */
void servos_telemetry_mavlink_send(servos_telemetry_t* servos_telemetry, Mavlink_stream* Mavlink_stream, mavlink_message_t* msg);

#endif /* SERVOS_TELEMETRY_H_ */