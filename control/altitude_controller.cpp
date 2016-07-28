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
 * \file altitude_controller.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   A simple altitude controller for copter
 *
 ******************************************************************************/


#include "control/altitude_controller.hpp"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Altitude_controller::Altitude_controller(const position_command_t& position_command, 
                                         const altitude_t& altitude, 
                                         thrust_command_t& thrust_command,
                                         altitude_controller_conf_t config):
    position_command_(position_command),
    altitude_(altitude),
    thrust_command_(thrust_command)
{
    hover_point_ = config.hover_point;
    pid_controller_init(&pid_, &config.pid_config);
}


bool Altitude_controller::init(void)
{   
    return true;
}


bool Altitude_controller::update(void)
{
    float error = 0.0f;

    switch (position_command_.mode)
    {
        case POSITION_COMMAND_MODE_LOCAL:
            error = position_command_.xyz[2] - (-altitude_.above_ground);
            break;

        case POSITION_COMMAND_MODE_GLOBAL:
            error = position_command_.xyz[2] - (-altitude_.above_sea);
            break;
    }

    thrust_command_.thrust = hover_point_ - pid_controller_update(&pid_, error);

    return true;
}