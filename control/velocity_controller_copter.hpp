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
 * \file velocity_controller_copter.hpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 * \author Basil Huber
 *
 * \brief A velocity controller for copters.
 *
 * \details It takes a velocity command as input and computes an attitude
 * command as output. It is best suited for a hovering UAV with the thrust along
 * its local -Z axis (like multicopters).
 *
 ******************************************************************************/


#ifndef VELOCITY_CONTROLLER_COPTER_HPP_
#define VELOCITY_CONTROLLER_COPTER_HPP_

#include "control/velocity_controller.hpp"
#include "control/pid_controller.hpp"
#include "sensing/ahrs.hpp"
#include "sensing/ins.hpp"

/**
 * \brief Velocity controller for hovering platforms
 */
class Velocity_controller_copter : public Velocity_controller
{
public:

    /**
     * \brief Velocity controller configuration
     */
    struct conf_t: public Velocity_controller::conf_t
    {
        float thrust_hover_point;
    };


    /**
    * \brief   Default Configuration
    *
    * /return  config
    */
    static inline conf_t default_config(void);


    /**
     * \brief                       Constructor
     *
     * \param   ahrs                Reference to estimated attitude
     * \param   ins                 Reference to estimated speed and position
     * \param   config              Configuration
     */
    Velocity_controller_copter(const ahrs_t& ahrs, const INS& ins, const conf_t& config = default_config());


protected:

    /**
     * \brief   Computes attitude and thrust command based on desired acceleration. Depends on robot dynamics
     *
     * \param   accel_vector        Desired acceleration vectors (input)
     * \param   attitude_command    Attitude command (output)
     * \param   thrust_command      Thrust command (output)
     *
     * \return  success
     */
    virtual bool compute_attitude_and_thrust_from_desired_accel(const std::array<float,3>& accel_command,
                                                                attitude_command_t& attitude_command,
                                                                thrust_command_t& thrust_command);

    float thrust_hover_point_;        ///< Amount of thrust required to hover (between -1 and 1)
};


Velocity_controller_copter::conf_t Velocity_controller_copter::default_config(void)
{
    conf_t conf = {};

    conf.thrust_hover_point                    = -0.26f;

    // -----------------------------------------------------------------
    // ------ X PID ----------------------------------------------------
    // -----------------------------------------------------------------
    conf.pid_config[X]                         = {};
    conf.pid_config[X].p_gain                  = 0.2f;
    conf.pid_config[X].clip_min                = -0.5f;
    conf.pid_config[X].clip_max                = 0.5f;
    conf.pid_config[X].integrator              = {};
    conf.pid_config[X].integrator.gain         = 0.0125f;
    conf.pid_config[X].integrator.clip_pre     = 1.0f;
    conf.pid_config[X].integrator.accumulator  = 0.0f;
    conf.pid_config[X].integrator.clip         = 0.5f;
    conf.pid_config[X].differentiator          = {};
    conf.pid_config[X].differentiator.gain     = 0.0f;
    conf.pid_config[X].differentiator.previous = 0.0f;
    conf.pid_config[X].differentiator.clip     = 1.0f;
    conf.pid_config[X].soft_zone_width         = 0.0f;

    // -----------------------------------------------------------------
    // ------ Y PID ----------------------------------------------------
    // -----------------------------------------------------------------
    conf.pid_config[Y]                         = {};
    conf.pid_config[Y].p_gain                  = 0.15f;
    conf.pid_config[Y].clip_min                = -0.5f;
    conf.pid_config[Y].clip_max                = 0.5f;
    conf.pid_config[Y].integrator              = {};
    conf.pid_config[Y].integrator.gain         = 0.0125f;
    conf.pid_config[Y].integrator.clip_pre     = 1.0f;
    conf.pid_config[Y].integrator.accumulator  = 0.0f;
    conf.pid_config[Y].integrator.clip         = 0.5f;
    conf.pid_config[Y].differentiator          = {};
    conf.pid_config[Y].differentiator.gain     = 0.0f;
    conf.pid_config[Y].differentiator.previous = 0.0f;
    conf.pid_config[Y].differentiator.clip     = 1.0f;
    conf.pid_config[Y].soft_zone_width         = 0.0f;

    // ---------------------------------------------------------------------
    // ------ Z PID --------------------------------------------------------
    // ---------------------------------------------------------------------
    conf.pid_config[Z]                         = {};
    conf.pid_config[Z].p_gain                  = 0.20f;
    conf.pid_config[Z].clip_min                = -0.9f;
    conf.pid_config[Z].clip_max                = 0.65f;
    conf.pid_config[Z].integrator              = {};
    conf.pid_config[Z].integrator.gain         = 0.01f;
    conf.pid_config[Z].integrator.clip_pre     = 2.0f;
    conf.pid_config[Z].integrator.accumulator  = 0.0f;
    conf.pid_config[Z].integrator.clip         = 0.3f;
    conf.pid_config[Z].differentiator          = {};
    conf.pid_config[Z].differentiator.gain     = 0.08f;
    conf.pid_config[Z].differentiator.previous = 0.0f;
    conf.pid_config[Z].differentiator.clip     = 0.04f;
    conf.pid_config[Z].soft_zone_width         = 0.2f;

    return conf;
}

#endif /* VELOCITY_CONTROLLER_COPTER_HPP_ */
