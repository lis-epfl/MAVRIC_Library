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
 * \file simulation.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Simulation
 *
 ******************************************************************************/


#include "simulation/simulation.hpp"
#include "hal/common/time_keeper.hpp"



Simulation::Simulation(Dynamic_model& dynamic_model, simulation_conf_t config):
    dynamic_model_(dynamic_model),
    accelerometer_(dynamic_model_),
    gyroscope_(dynamic_model_),
    magnetometer_(dynamic_model_),
    barometer_(dynamic_model_),
    sonar_(dynamic_model_),
    gps_(dynamic_model_),
    last_update_us_(0.0f)
{}


bool Simulation::update(void)
{
    bool success = true;

    success &= dynamic_model_.update();
    last_update_us_ = time_keeper_get_us();

    return success;
}


const float& Simulation::last_update_us(void) const
{
    return last_update_us_;
}


Accelerometer& Simulation::accelerometer(void)
{
    return accelerometer_;
}


Gyroscope& Simulation::gyroscope(void)
{
    return gyroscope_;
}


Magnetometer& Simulation::magnetometer(void)
{
    return magnetometer_;
}


Barometer& Simulation::barometer(void)
{
    return barometer_;
}


Sonar& Simulation::sonar(void)
{
    return sonar_;
}


Gps& Simulation::gps(void)
{
    return gps_;
}
