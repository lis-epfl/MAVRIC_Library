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
 * \file barometer_sim.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Simulation for barometers
 *
 ******************************************************************************/


#ifndef BAROMETER_SIM_HPP_
#define BAROMETER_SIM_HPP_

#include "drivers/barometer.hpp"
#include "simulation/dynamic_model.hpp"

/**
 * \brief   Simulation for barometers
 */
class Barometer_sim: public Barometer
{
public:
    /**
     * \brief   Constructor
     *
     * \param   dynamic_model   Reference to dynamic model
     */
    Barometer_sim(Dynamic_model& dynamic_model);


    /**
     * \brief   Initialise the sensor
     *
     * \return  Success
     */
    bool init(void);


    /**
     * \brief   Main update function
     * \detail  Reads new values from sensor
     *
     * \return  Success
     */
    bool update(void);


    /**
    * \brief   Get the last update time in microseconds
    *
    * \return   Value
    */
    uint64_t last_update_us(void) const;


    /**
     * \brief   Return the pressure
     *
     * \return  Value
     */
    float pressure(void)  const;


    /**
     * \brief   Get the altitude in meters above sea level
     *
     * \detail  Global frame: (>0 means upward)
     *
     * \return  Value
     */
    float altitude_gf(void) const;


    /**
     * \brief   Get the altitude in meters above sea level (not filtered)
     *
     * \detail  Global frame: (>0 means upward)
     *
     * \return  Value
     */
    float altitude_gf_raw(void) const;


    /**
     * \brief   Get the vertical speed in meters/second
     *
     * \detail  NED frame: (>0 means downward)
     *
     * \return  Value
     */
    float vertical_speed_lf(void) const;


    /**
     * \brief   Get the vertical speed in meters/second (not filtered)
     *
     * \detail  NED frame: (>0 means downward)
     *
     * \return  Value
     */
    float vertical_speed_lf_raw(void) const;


    /**
     * \brief   Get sensor temperature
     *
     * \return  Value
     */
    float temperature(void) const;

private:
    Dynamic_model&  dynamic_model_; ///< Reference to dynamic model

    uint64_t last_update_us_;   ///< Last update time
    float pressure_;            ///< Measured pressure
    float temperature_;         ///< Measured temperature
    float altitude_gf_;         ///< Measured altitude (global frame)
    float altitude_filtered;    ///< Measured altitude without bias removal
    float speed_lf_;            ///< Vario altitude speed (ned frame)
};

#endif /* BAROMETER_SIM_HPP_ */
