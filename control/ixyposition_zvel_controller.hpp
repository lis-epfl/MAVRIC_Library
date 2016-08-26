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
 * \file ixyposition_zvel_controller.hpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief Interface for horizontal position and vertical velocity controller (used e.g. for landing)
 *
 ******************************************************************************/


#ifndef IXYPOSITION__ZVEL_CONTROLLER_HPP_
#define IXYPOSITION__ZVEL_CONTROLLER_HPP_

#include "util/coord_conventions.hpp"
#include "control/control_command.h"

class IXyposition_zvel_controller
{
public:
    /*
     * \brief   structure representing a horizontal position & vertical velocity command; contains desired horizontal position and vertical velocity in local frame
     */
    struct xypos_zvel_command_t : base_command_t
    {
        float pos_x;    ///< desired position in x in local frame
        float pos_y;    ///< desired position in y in local frame
        float vel_z;    ///< desired velocity in z in local frame
    };

    /**
     * \brief   Update controller;
     */
    virtual void update() = 0;

    /**
     * \brief           sets the horizontal position & vertical velocity command
     *
     * \param command   xy position z velocity command indicating target location & velocity in local frame
     *
     * \return success  whether command was accepted
     */
    virtual bool set_xyposition_zvel_command(const xypos_zvel_command_t& command) = 0;
};

#endif /* IXYPOSITION__ZVEL_CONTROLLER_HPP_ */