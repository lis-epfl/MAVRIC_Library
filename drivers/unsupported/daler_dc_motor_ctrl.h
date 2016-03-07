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
 * \file dc_motor_ctrl.h
 *
 * \author MAV'RIC Team
 * \author Ludovic Daler
 *
 * \brief This file configures the dc_motor_ctrl UART communication
 *
 ******************************************************************************/


#ifndef DC_MOTOR_CTRL_H_
#define DC_MOTOR_CTRL_H_

#include "util/streams.h"
#include "buffer.h"
#include "mavlink_stream.h"

/**
 * \brief structure of the i2cxl_sonar module
*/
typedef struct
{
    buffer_t dc_motor_ctrl_in_buffer;           ///< The dc_motor_ctrl incoming buffer
    byte_stream_t dc_motor_ctrl_out_stream;     ///< The dc_motor_ctrl outgoing byte stream
    byte_stream_t dc_motor_ctrl_in_stream;      ///< The dc_motor_ctrl incoming byte stream
    float wingrons_angle[2];                    ///< Angles wanted for the wingrons dc_motors
    float wingrons_speed[2];                    ///< Wanted speed for the wingrons dc_motors
    const mavlink_stream_t* mavlink_stream;     ///< Pointer to mavlink stream
} daler_dc_motor_ctrl_t;


/**
 * \brief Initialize the dc_motor_ctrl module
 *
 * \param dc_motor_ctrl pointer to DC motor controller structure
 * \param UID uart device number
 *
 * \return  success
 */
bool daler_dc_motor_ctrl_init(daler_dc_motor_ctrl_t* dc_motor_ctrl, int32_t UID);


/**
 * \brief Update the dc_motor_ctrl module
 *
 * \param dc_motor_ctrl pointer to DC motor controller structure
 *
 * \return success
 */
bool daler_dc_motor_ctrl_update(daler_dc_motor_ctrl_t* dc_motor); //, const float wingrons[2] );

#endif /* DC_MOTOR_CTRL_H_ */