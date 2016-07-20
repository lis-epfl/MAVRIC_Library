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
 * \file state_display.hpp
 *
 * \author MAV'RIC Team
 * \author Jean-François Burnier
 *
 * \brief Interface class for state display
 *
 ******************************************************************************/

#ifndef STATE_DISPLAY_HPP_
#define STATE_DISPLAY_HPP_

#include "communication/mav_modes.hpp"

/**
 * \brief   Interface class for state display
 */
class State_display
{
	public:
	 /**
     * \brief   Main update function
     * \detail  Displays state 
     *
     * \return  Success
     */
    virtual bool update(void) = 0;

     /**
     * \brief   Set state function
     * \detail  Set the state to be displayed
     * \param   state_new  new state to display
     *
     * \return  Success
     */
    bool set_state(const mav_state_t state_new);

	protected:
		mav_state_t state_;
		mav_state_t state_old_;
        uint8_t idle_;

};

/**
 * \brief  Glue method for scheduler
 */
static inline bool task_state_display_update(State_display* state_display)
{
    return state_display->update();
};


#endif /* STATE_DISPLAY_HPP_ */