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
 * \file irate_controller.hpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief Interface for rate controller
 *
 ******************************************************************************/


#ifndef IRATE_CONTROLLER_HPP_
#define IRATE_CONTROLLER_HPP_

#include "util/coord_conventions.hpp"
#include "control/control_command.h"

class IRate_controller
{
public:
    /*
     * \brief   structure representing a rate command; contains desired angular velocity and thrust in body frame
     */
    struct rate_command_t : base_command_t
    {
        std::array<float,3>  rates;       ///< desired rates (angular velocity) in body frame
        float                thrust;     ///< desired thrust
    };

    /**
     * \brief   Update controller;
     */
    virtual void update() = 0;

    /**
     * \brief           sets the rate command (desired angular velocity and thrust)
     * \param command   rate command indicating desired angular velocity and thrust in body frame
     */
    inline virtual void set_rate_command(const rate_command_t& command) = 0;
};

#endif /* IRATE_CONTROLLER_HPP_ */