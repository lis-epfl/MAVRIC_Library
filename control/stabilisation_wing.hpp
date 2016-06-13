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
 * \file stabilisation_wing.hpp
 *
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *
 * \brief This file handles the stabilization of the platform
 *
 ******************************************************************************/


#ifndef STABILISATION_WING_H_
#define STABILISATION_WING_H_

#include "control/navigation.hpp"
#include "sensing/imu.hpp"
#include "sensing/position_estimation.hpp"
#include "drivers/airspeed_analog.hpp"

extern "C"
{
#include "control/stabilisation.h"
#include "control/control_command.h"
}


/**
 * \brief Structure containing the stacked controller
 */
typedef struct
{
    stabiliser_t rate_stabiliser;                               ///< The rate controller structure
    stabiliser_t attitude_stabiliser;                           ///< The attitude controller structure
    stabiliser_t velocity_stabiliser;                           ///< The velocity controller structure
    stabiliser_t position_stabiliser;                           ///< The position controller structure
} stabiliser_stack_wing_t;

/**
 * \brief Structure containing the pointers to the data needed in this module
 */
typedef struct
{
    stabiliser_stack_wing_t stabiliser_stack;                   ///< The pointer to the PID parameters values for the stacked controller
    control_command_t* controls;                                ///< The pointer to the control structure
    torque_command_t* torque_command;                           ///< The pointer to the torque command (output)
    thrust_command_t* thrust_command;                           ///< The pointer to the thrust command (output)
    const Imu* imu;                                             ///< The pointer to the IMU structure
    const ahrs_t* ahrs;                                         ///< The pointer to the attitude estimation structure
    const Position_estimation* pos_est;                         ///< The pointer to the position estimation structure
    const Airspeed_analog* airspeed_analog;                     ///< The pointer to the analog airspeed sensor structure
    const Navigation* navigation;                               ///< The pointer to the navigation structure
    const Gps* gps;                                             ///< The pointer to the GPS structure
    float thrust_apriori;                                       ///< A priori on the thrust for velocity control
    float pitch_angle_apriori;                                  ///< Constant a priori on the pitch angle
    float pitch_angle_apriori_gain;                             ///< Gain of the pitch angle a priori which is function of the roll value
    float max_roll_angle;                                       ///< Maximum roll value that the velocity layer could ask to follow
    float take_off_thrust;                                      ///< Thrust value used during the take-off
    float take_off_pitch;                                       ///< Pitch angle used during the take-off
    float landing_pitch;                                        ///< Pitch angle used during the landing
    float landing_max_roll;                                     ///< Maximum roll angle during landing
    float dt_s;                                                 ///< Time interval between to updates
    float last_update_s;                                        ///< Last update in seconds
} stabilisation_wing_t;

/**
 * \brief Structure containing the configuration data
 */
typedef struct
{
    float thrust_apriori;                                       ///< A priori thrust
    float pitch_angle_apriori;                                  ///< Constant a priori on the pitch angle
    float pitch_angle_apriori_gain;                             ///< Gain of the pitch angle a priori which is function of the roll value
    float max_roll_angle;                                       ///< Maximum roll value that the velocity layer could ask to follow
    float take_off_thrust;                                      ///< Thrust value used during the take-off
    float take_off_pitch;                                       ///< Pitch angle used during the take-off
    float landing_pitch;                                        ///< Pitch angle used during the landing
    float landing_max_roll;                                     ///< Maximum roll angle during landing
    stabiliser_stack_wing_t stabiliser_stack;                   ///< The pointer to the PID parameters values and output for the stacked controller
} stabilisation_wing_conf_t;

/**
 * \brief                           Initialize module stabilization
 *
 * \param   stabilisation_wing      The pointer to the stabilisation wing structure
 * \param   stabiliser_conf         The structure with all PID controllers
 * \param   controls                The pointer to the controlling inputs
 * \param   torque_command          The pointer to the torque command (output)
 * \param   thrust_command          The pointer to the thrust command (output)
 * \param   imu                     The pointer to the IMU structure
 * \param   ahrs                    The pointer to the attitude estimation structure
 * \param   pos_est                 The pointer to the position estimation structure
 * \param   airspeed_analog         The pointer to the analog airspeed sensor structure
 * \param   navigation              The pointer to the navigation structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool stabilisation_wing_init(stabilisation_wing_t* stabilisation_wing, const stabilisation_wing_conf_t stabiliser_conf, control_command_t* controls, torque_command_t* torque_command, thrust_command_t* thrust_command, const Imu* imu, const ahrs_t* ahrs, const Position_estimation* pos_est, const Airspeed_analog* airspeed_analog, const Navigation* navigation, const Gps* gps);

/**
 * \brief                       Main Controller for controlling and stabilizing the wing
 *
 * \param   stabilisation_wing      The stabilisation structure
 */
void stabilisation_wing_cascade_stabilise(stabilisation_wing_t* stabilisation_wing);


#endif /* STABILISATION_WING_H_ */
