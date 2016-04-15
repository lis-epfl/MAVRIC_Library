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
 * \file sonar_sim.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Simulation for sonars
 *
 ******************************************************************************/


#ifndef SONAR_SIM_HPP_
#define SONAR_SIM_HPP_


#include "drivers/sonar.hpp"
#include "simulation/dynamic_model.hpp"


/**
 * \brief   Configuration structure for Sonar_sim
 */
typedef struct
{
    float               min_distance;       ///< Minimum distance the sensor can read
    float               max_distance;       ///< Maximum distance the sensor can read
    std::array<float, 3>    orientation_bf; ///< Sensor orientation relative to the body frame
} sonar_sim_conf_t;


/**
 * \brief   Default configuration
 *
 * \return  Config structure
 */
static inline sonar_sim_conf_t sonar_sim_default_config();


/**
 * \brief   Simulation for sonars
*/
class Sonar_sim: public Sonar
{
public:
    /**
     * \brief   Constructor
     *
     * \param   dynamic_model   Reference to dynamic model
     * \param   config          Configuration
     */
    Sonar_sim(Dynamic_model& dynamic_model, sonar_sim_conf_t config = sonar_sim_default_config());


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
     * \brief   Get last update time in microseconds
     *
     * \return  Update time
     */
    const float& last_update_us(void) const;


    /**
     * \brief   Get sensor orientation relative to the platform (in body frame)
     *
     * \return  3D orientation
     */
    const std::array<float, 3>& orientation_bf(void) const;


    /**
     * \brief   Get latest distance measure
     *
     * \return  Value
     */
    const float& distance(void) const;


    /**
     * \brief   Get velocity estimate from consecutive measurements
     *
     * \return  Value
     */
    const float& velocity(void) const;


    /**
     * \brief   Indicates whether the measurements can be trusted
     *
     * \return  Value
     */
    const bool& healthy(void) const;

private:
    Dynamic_model&      dynamic_model_; ///< Reference to dynamic model

    sonar_sim_conf_t    config_;        ///< Configuration
    float               distance_;      ///< Current distance
    float               velocity_;      ///< Current velocity
    float               last_update_us_;///< Last update time in microseconds
    bool                healthy_;       ///< Sensor status
};


/**
 * \brief   Default configuration
 *
 * \return  Config structure
 */
static inline sonar_sim_conf_t sonar_sim_default_config()
{
    sonar_sim_conf_t conf = {};

    // Correct range between 20cm and 7m, - safety
    conf.min_distance = 0.20f;
    conf.max_distance = 5.0f;

    // Default orientation is looking downwards (NED)
    conf.orientation_bf = std::array<float, 3> {{0.0f, 0.0f, 1.0f}};

    return conf;
}


#endif /* SONAR_SIM_HPP_ */
