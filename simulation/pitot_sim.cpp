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
 * \file pitot_sim.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Nicolas Jacquemin
 *
 * \brief Simulated pitot
 *
 ******************************************************************************/


#include "simulation/pitot_sim.hpp"

extern "C"
{
#include "util/constants.h"
}

Pitot_sim::Pitot_sim(Dynamic_model& dynamic_model):
    dynamic_model_(dynamic_model),
    speed_(0.0f)
{}


bool Pitot_sim::init(void)
{
    return true;
}


bool Pitot_sim::update(void)
{
    bool success = true;

    // Update dynamic model
    success &= dynamic_model_.update();

    // Get velocity
    speed_ = dynamic_model_.x_speed_pitot();

    return success;
}


const float& Pitot_sim::last_update_us(void) const
{
    return dynamic_model_.last_update_us();
}


const float& Pitot_sim::pitot(void) const
{
    return speed_;
}
