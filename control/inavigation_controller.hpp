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
 * \file inavigation_controller.hpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief Interface for navigation controller
 *
 ******************************************************************************/


#ifndef INAVIGATION_CONTROLLER_HPP_
#define INAVIGATION_CONTROLLER_HPP_

#include "util/coord_conventions.hpp"
#include "control/control_command.h"

class INavigation_controller
{
public:
    /*
     * \brief   structure representing a navigation command; contains desired position in local frame
     */
    struct nav_command_t : base_command_t
    {
        local_position_t    pos;        ///< desired position in local frame
    };

    /**
     * \brief   Update controller;
     */
    virtual void update() = 0;

    /**
     * \brief           sets the navigation command (desired position)
     * \param command   navigation command indicating navigation target location in local frame
     */
    inline virtual void set_navigation_command(const nav_command_t& command) = 0;
};

#endif /* INAVIGATION_CONTROLLER_HPP_ */