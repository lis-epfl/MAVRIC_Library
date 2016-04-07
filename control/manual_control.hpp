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
 * \file manual_control.h
 *
 * \author MAV'RIC Team
 *
 * \brief This module takes care of taking the correct input for the control
 * (i.e. the remote or the joystick)
 *
 ******************************************************************************/


#ifndef MANUAL_CONTROL_H_
#define MANUAL_CONTROL_H_

#include "communication/remote.hpp"
#include "control/joystick.hpp"
#include "communication/state.hpp"

extern "C"
{
#include "control/stabilisation.h"
}

/**
 * \brief   The source mode enum
 */
typedef enum : int32_t
{
    MODE_SOURCE_REMOTE      = 0,
    MODE_SOURCE_GND_STATION = 1,
    MODE_SOURCE_JOYSTICK    = 2,
} mode_source_t;


/**
 * \brief   Control source
 */
typedef enum : int32_t
{
    CONTROL_SOURCE_REMOTE       = 0,
    CONTROL_SOURCE_NONE         = 1,
    CONTROL_SOURCE_JOYSTICK     = 2,
} control_source_t;


/**
 * \brief Configuration for manual control
 */
typedef struct
{
    mode_source_t       mode_source;        ///< The source mode
    control_source_t    control_source;     ///< Flag to tell whether the remote is active or not
} manual_control_conf_t;


/**
 * \brief The manual control structure
 */
struct manual_control_t
{
    /**
     * \brief                   Constructor
     *
     * \param   config          The pointer to the configuration structure of the module
     * \param   remote_config   The pointer to the remote structure
     */
    manual_control_t(Satellite* sat, manual_control_conf_t config, remote_conf_t remote_config);


    mode_source_t           mode_source;        ///< The source mode
    control_source_t        control_source;     ///< Flag to tell whether the remote is active or not

    remote_t                remote;             ///< The pointer to the remote structure
    joystick_t              joystick;           ///< The pointer to the joystick structure
} ;


/**
 * \brief   Selects the source input for the attitude command
 *
 * \param   manual_control  The pointer to the manual control structure
 * \param   controls        The pointer to the command structure that will be executed
 */
void manual_control_get_control_command(manual_control_t* manual_control, control_command_t* controls);


/**
 * \brief   Selects the source input for the velocity command
 *
 * \param   manual_control  The pointer to the manual control structure
 * \param   controls        The pointer to the command structure that will be executed
 */
void manual_control_get_velocity_vector(manual_control_t* manual_control, control_command_t* controls);


/**
 * \brief   Selects the source input and returns the thrust
 *
 * \param   manual_control  The pointer to the manual control structure
 *
 * \return  The value of the thrust depending on the source input
 */
float manual_control_get_thrust(const manual_control_t* manual_control);


/**
 * \brief   Compute torque command from the manual input
 *
 * \param   manual_control  Manual control structure (input)
 * \param   command         Torque command (output)
 * \param   scale           Scale (maximum output / max remote input)
 */
void manual_control_get_torque_command(const manual_control_t* manual_control, torque_command_t* command, float scale);


/**
 * \brief   Compute rate command from the manual input
 *
 * \param   manual_control  Manual control structure (input)
 * \param   command         Rate command (output)
 * \param   scale           Scale (maximum output / max remote input)
 */
void manual_control_get_rate_command(const manual_control_t* manual_control, rate_command_t* command, float scale);


/**
 * \brief   Compute thrust command from the manual input
 *
 * \param   manual_control  Manual control structure (input)
 * \param   command         Thrust command (output)
 */
void manual_control_get_thrust_command(const manual_control_t* manual_control, thrust_command_t* command);


/**
 * \brief   Compute attitude command from the manual input (absolute angles)
 *
 * \param   manual_control  Manual control structure (input)
 * \param   command         Attitude command (output)
 * \param   scale           Scale (maximum output / max remote input)
 */
void manual_control_get_attitude_command_absolute_yaw(const manual_control_t* manual_control, attitude_command_t* command, float scale);


/**
 * \brief   Compute attitude command from the manual input (absolute roll and pitch, integrated yaw)
 *
 * \param   manual_control  Manual control structure (input)
 * \param   k_yaw           Integration factor for yaw (0.02 is ok) (input)
 * \param   command         Attitude command (output)
 * \param   scale           Scale (maximum output / max remote input)
 */
void manual_control_get_attitude_command(const manual_control_t* manual_control, const float k_yaw, attitude_command_t* command, float scale);


/**
 * \brief   Compute attitude command from the manual input (absolute roll and pitch, integrated yaw)
 *
 * \param   manual_control  Manual control structure (input)
 * \param   ki_yaw          Integration factor for yaw (0.02 is ok) (input)
 * \param   command         Attitude command (output)
 * \param   scale           Scale (maximum output / max remote input)
 * \param   reference_pitch Transition factor (0: forward flight, PI/2:hover)
 */
void manual_control_get_attitude_command_vtol(const manual_control_t* manual_control, const float ki_yaw, attitude_command_t* command, float scale, float reference_pitch);


/**
 * \brief   Compute velocity command from the manual input
 *
 * \param   manual_control  Manual control structure (input)
 * \param   command         Velocity command (output)
 * \param   scale           Scale (maximum output / max remote input)
 */
void manual_control_get_velocity_command(const manual_control_t* manual_control, velocity_command_t* command, float scale);


/**
 * \brief   Returns the value of the mode from the desired source input
 *
 * \param   manual_control          The pointer to the manual_control structure
 * \param   mode_current            The current mode of the MAV
 *
 * \return  The value of the mode
 */
mav_mode_t manual_control_get_mode_from_source(manual_control_t* manual_control, mav_mode_t mode_current);



/**
 * \brief   Returns the quality of the strength of the remote receiver
 *
 * \param   manual_control          The pointer to the manual_control structure
 *
 * \return  The current status of the remote controller
 */
signal_quality_t manual_control_get_signal_strength(manual_control_t* manual_control);


#endif /* MANUAL_CONTROL_H_ */
