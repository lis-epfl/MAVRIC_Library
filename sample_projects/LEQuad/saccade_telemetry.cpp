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
 * \file saccade_telemetry.cpp
 *
 * \author MAV'RIC Team
 * \author Darius Merk
 *
 * \brief Saccade telemetry
 *
 ******************************************************************************/


#include "saccade_telemetry.hpp"
#include "hal/common/time_keeper.hpp"
extern "C"
{
#include "util/maths.h"
#include "util/constants.h"
}



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------



void saccade_telemetry_send_vector(const Saccade_controller* Saccade_controller, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_debug_vect_pack(mavlink_stream->sysid(),
                                 mavlink_stream->compid(),
                                 msg,
                                 "CAN and CAD",
                                 time_keeper_get_us(),
                                 Saccade_controller->can_,
                                 Saccade_controller->cad_,
                                 Saccade_controller->saccade_state_ );
                                 // 1000*(Saccade_controller->derotation_constant_*Saccade_controller->ahrs_.angular_speed[2]),
                                 // (Saccade_controller->flow_front_.of.x[20] - 1000*(Saccade_controller->derotation_constant_*Saccade_controller->ahrs_.angular_speed[2])));

                                 // Saccade_controller->can_,
                                 // Saccade_controller->cad_,
                                 // 0);
}
