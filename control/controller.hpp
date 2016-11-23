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
 * \file controller.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Interface for controllers
 *
 ******************************************************************************/

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "control/control_command.hpp"

template<typename in_command_T, typename out_command_T = empty_command_t>
class Controller
{
public:
    typedef in_command_T in_command_t;
    typedef out_command_T out_command_t;

    /**
     * \brief   Main update function
     *
     * \return  success
     */
    virtual bool update(void) = 0;


    /**
     * \brief   Sets the input command
     *
     * \param   command   Input command
     *
     * \return  success
     */
    virtual bool set_command(const in_command_t& command) = 0;


    /**
     * \brief   Returns the input command
     *
     * \param   command   Input command
     *
     * \return  success
     */
    virtual bool get_command(in_command_t& command) const = 0;


    /**
     * \brief   Returns the output command
     *
     * \param   command   output command
     *
     * \return  success
     */
    virtual bool get_output(out_command_t& command) const = 0;
};

#endif  // CONTROLLER_HPP_
