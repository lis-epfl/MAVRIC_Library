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
 * \file manual_control.hpp
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

/* forward declaration for friend function */
class Onboard_parameters;

/**
 * \brief The manual control structure
 */
class Manual_control
{
friend bool mavlink_telemetry_add_onboard_parameters(Onboard_parameters* onboard_parameters, Central_data* central_data);

public:
    /**
 * \brief   The source mode enum
 */
    enum mode_source_t : int32_t
{
    MODE_SOURCE_REMOTE      = 0,
    MODE_SOURCE_GND_STATION = 1,
    MODE_SOURCE_JOYSTICK    = 2,
    };


/**
 * \brief   Control source
 */
    enum control_source_t : int32_t
{
    CONTROL_SOURCE_REMOTE       = 0,
    CONTROL_SOURCE_NONE         = 1,
    CONTROL_SOURCE_JOYSTICK     = 2,
    };

/**
 * \brief Configuration for manual control
 */
    struct conf_t
{
    mode_source_t       mode_source;        ///< The source mode
    control_source_t    control_source;     ///< Flag to tell whether the remote is active or not
    };


/**
     * \brief                   Constructor
 *
 * \param   config          The pointer to the configuration structure of the module
 * \param   remote_config   The pointer to the remote structure
 */
    Manual_control(Satellite* sat, conf_t config, remote_conf_t remote_config);

/**
 * \brief   Selects the source input for the attitude command
 *
 * \param   controls        The pointer to the command structure that will be executed
 */
    void get_control_command(control_command_t* controls);


/**
 * \brief   Selects the source input for the velocity command
 *
 * \param   controls        The pointer to the command structure that will be executed
 */
    void get_velocity_vector(control_command_t* controls);


    /**
     * \brief   Selects the source input for the rate command for the wing
     *
     * \param   controls        The pointer to the command structure that will be executed
     */
    void get_rate_command_wing(control_command_t* controls);

    /**
     * \brief   Get the input from the joystick (Symbiotic project)
     *
   	 * \param   controls        The pointer to the command structure that will be executed
   	 */
    void manual_control_get_from_joystick_symbiotic(control_command_t* controls);

/**
     * \brief   Selects the source input for the attitude command for the wing
     *
     * \param   controls        The pointer to the command structure that will be executed
     */
    void get_angle_command_wing(control_command_t* controls);

    /**
     * \brief   Selects the source input for the velocity command for the wing
     *
     * \param   ki_yaw          The yaw integrator gain
     * \param   controls        The pointer to the command structure that will be executed
     */
    void get_velocity_vector_wing(const float ki_yaw, control_command_t* controls);


    /**
 * \brief   Selects the source input and returns the thrust
 *
 * \return  The value of the thrust depending on the source input
 */
    float get_thrust() const;


/**
 * \brief   Compute torque command from the manual input
 *
 * \param   command         Torque command (output)
 * \param   scale           Scale (maximum output / max remote input)
 */
    void get_torque_command(torque_command_t* command, float scale) const;


/**
 * \brief   Compute rate command from the manual input
 *
 * \param   command         Rate command (output)
 * \param   scale           Scale (maximum output / max remote input)
 */
    void get_rate_command(rate_command_t* command, float scale) const;


/**
 * \brief   Compute thrust command from the manual input
 *
 * \param   command         Thrust command (output)
 */
    void get_thrust_command(thrust_command_t* command) const;


/**
 * \brief   Compute attitude command from the manual input (absolute angles)
 *
 * \param   command         Attitude command (output)
 * \param   scale           Scale (maximum output / max remote input)
 */
    void get_attitude_command_absolute_yaw(attitude_command_t* command, float scale) const;


/**
 * \brief   Compute attitude command from the manual input (absolute roll and pitch, integrated yaw)
 *
 * \param   k_yaw           Integration factor for yaw (0.02 is ok) (input)
 * \param   command         Attitude command (output)
 * \param   scale           Scale (maximum output / max remote input)
 */
    void get_attitude_command(const float k_yaw, attitude_command_t* command, float scale) const;


/**
 * \brief   Compute attitude command from the manual input (absolute roll and pitch, integrated yaw)
 *
 * \param   ki_yaw          Integration factor for yaw (0.02 is ok) (input)
 * \param   command         Attitude command (output)
 * \param   scale           Scale (maximum output / max remote input)
 * \param   reference_pitch Transition factor (0: forward flight, PI/2:hover)
 */
    void get_attitude_command_vtol(const float ki_yaw, attitude_command_t* command, float scale, float reference_pitch) const;


/**
 * \brief   Compute velocity command from the manual input
 *
 * \param   command         Velocity command (output)
 * \param   scale           Scale (maximum output / max remote input)
 */
    void get_velocity_command(velocity_command_t* command, float scale) const;


/**
 * \brief   Returns the value of the mode from the desired source input
 *
 * \param   mode_current            The current mode of the MAV
 *
 * \return  The value of the mode
 */
    mav_mode_t get_mode_from_source(mav_mode_t mode_current);

/**
 * \brief   Returns the value of the mode from the remote
 *
 * \param   manual_control          The pointer to the manual_control structure
 * \param   mode_current            The current mode of the MAV
 *
 * \return  The value of the mode
 */
mav_mode_t manual_control_get_mode_from_remote(mav_mode_t mode_current);

    /**
     * \brief   Sets the internal state of the joystick
     *
     * \details only mode of joystick is overwritten, remote keeps its interal state
     *
     * \param   mode_current            The current mode of the MAV
     *
     */
    void set_mode_of_source(mav_mode_t mode_current);

/**
 * \brief   Returns the quality of the strength of the remote receiver
 *
 * \return  The current status of the remote controller
 */
    signal_quality_t get_signal_strength();

    /**
     * \brief   Set the mode source (which control source can change the mode (remote, joystick, groundstation))
     *
     * \param   mode_source         which control source can change the mode (remote, joystick, groundstation)
     */
    inline void set_mode_source(mode_source_t mode_source) {mode_source_ = mode_source;};

    /**
     * \brief   Return the mode source (which control source can change the mode (remote, joystick, groundstation))
     *
     * \return   mode_source         which control source can change the mode (remote, joystick, groundstation)
     */
    inline mode_source_t mode_source() const {return mode_source_;};    

    /**
     * \brief   Set the control source (which control source has control (remote, joystick, none))
     *
     * \param   control_source         which control source has control (remote, joystick, none)
     */
    inline void set_control_source(control_source_t control_source) {control_source_ = control_source;};

    /**
     * \brief   Return the control source (which control source has control (remote, joystick, none))
     *
     * \return   control_source         which control source has control (remote, joystick, none)
     */
    inline control_source_t control_source() const {return control_source_;};


    static inline conf_t default_config();



    remote_t                remote;             ///< The pointer to the remote structure
    joystick_t              joystick;           ///< The pointer to the joystick structure
private:
    mode_source_t           mode_source_;        ///< The source mode
    control_source_t        control_source_;     ///< Flag to tell whether the remote is active or not


} ;

Manual_control::conf_t Manual_control::default_config()
{
    conf_t conf = {};

    conf.mode_source = MODE_SOURCE_REMOTE;
    conf.control_source = CONTROL_SOURCE_REMOTE;

    return conf;
};





#endif /* MANUAL_CONTROL_H_ */
