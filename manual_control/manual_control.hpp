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


#ifndef MANUAL_CONTROL_HPP_
#define MANUAL_CONTROL_HPP_

#include "manual_control/remote.hpp"
#include "manual_control/joystick.hpp"
#include "status/state.hpp"


/* forward declaration for friend function */
class Onboard_parameters;

/**
 * \brief The manual control structure
 */
class Manual_control
{
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
        Joystick::conf_t    joystick_config;    ///< Configuration of the joystick
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
     * \brief   Selects the source input and returns the throttle
     *
     * \return  The value of the thrust depending on the source input
     */
    float throttle() const;


    /**
     * \brief   Selects the source input and returns the roll
     *
     * \return  The value of the thrust depending on the source input
     */
    float roll() const;


    /**
     * \brief   Selects the source input and returns the pitch
     *
     * \return  The value of the thrust depending on the source input
     */
    float pitch() const;


    /**
     * \brief   Selects the source input and returns the yaw
     *
     * \return  The value of the thrust depending on the source input
     */
    float yaw() const;


    /**
     * \brief   Compute torque command from the manual input
     *
     * \param   command         Torque command (output)
     * \param   scale_roll      Scale (maximum output / max remote input)
     * \param   scale_pitch     Scale (maximum output / max remote input)
     * \param   scale_yaw       Scale (maximum output / max remote input)
     */
    void get_torque_command(torque_command_t& command,
                            float scale_roll = 1.0f,
                            float scale_pitch = 1.0f,
                            float scale_yaw = 1.0f) const;


    /**
     * \brief   Compute rate command from the manual input
     *
     * \param   command         Rate command (output)
     * \param   scale_roll      Scale (maximum output / max remote input)
     * \param   scale_pitch     Scale (maximum output / max remote input)
     * \param   scale_yaw       Scale (maximum output / max remote input)
     */
    void get_rate_command(rate_command_t& command,
                          float scale_roll = 1.0f,
                          float scale_pitch = 1.0f,
                          float scale_yaw = 1.0f) const;


    /**
     * \brief   Compute thrust command from the manual input
     *
     * \param   command         Thrust command (output)
     * \param   scale           Scale
     */
    void get_thrust_command_copter(thrust_command_t& command, float scale = 1.0f) const;


    /**
     * \brief   Compute thrust command from the manual input
     *
     * \param   command         Thrust command (output)
     * \param   scale           Scale
     */
    void get_thrust_command_wing(thrust_command_t& command, float scale = 1.0f) const;


    /**
     * \brief   Compute attitude command from the manual input (absolute angles)
     *
     * \param   command         Attitude command (output)
     * \param   scale_roll      Scale (maximum output / max remote input)
     * \param   scale_pitch     Scale (maximum output / max remote input)
     * \param   scale_yaw       Scale (maximum output / max remote input)
     */
    void get_attitude_command_absolute_yaw( attitude_command_t& command,
                                            float scale_roll = 1.0f,
                                            float scale_pitch = 1.0f,
                                            float scale_yaw = 1.0f) const;


    /**
     * \brief   Compute attitude command from manual control source (absolute roll and pitch, relative yaw)
     * \details Yaw is relative to current yaw (command.yaw = current.yaw + 0.5 * input.yaw)
     *
     * \param   command             Attitude command (output)
     * \param   current_attitude    Current attitude of the vehicle
     * \param   scale_roll          Scale (maximum output / max remote input)
     * \param   scale_pitch         Scale (maximum output / max remote input)
     * \param   scale_yaw           Scale (maximum output / max remote input)
     *
     * \return  command
     */
    void get_attitude_command(  attitude_command_t& command,
                                const quat_t& current_attitude,
                                float scale_roll = 1.0f,
                                float scale_pitch = 1.0f,
                                float scale_yaw = 0.5f) const;


    /**
     * \brief   Compute attitude command from the manual input (absolute roll and pitch, integrated yaw)
     *
     * \param   command             Attitude command (output)
     * \param   current_attitude    Current attitude of the vehicle
     * \param   reference_pitch     Transition factor (0: forward flight, PI/2:hover)
     * \param   scale_roll          Scale (maximum output / max remote input)
     * \param   scale_pitch         Scale (maximum output / max remote input)
     * \param   scale_yaw           Scale (maximum output / max remote input)
     */
    void get_attitude_command_vtol( attitude_command_t& command,
                                    const quat_t& current_attitude,
                                    float reference_pitch,
                                    float scale_roll = 1.0f,
                                    float scale_pitch = 1.0f,
                                    float scale_yaw = 0.25f) const;


    /**
     * \brief   Compute velocity command from the manual input
     *
     * \param   command                     Velocity command (output)
     * \param   current_attitude            Current attitude of the vehicle
     * \param   current_velocity_command    Current velocity command
     * \param   scale_x         Scale (maximum output / max remote input)
     * \param   scale_y         Scale (maximum output / max remote input)
     * \param   scale_z         Scale (maximum output / max remote input)
     */
    void get_velocity_command_copter(velocity_command_t& command,
                                    const quat_t& current_attitude,
                                    const velocity_command_t& current_velocity_command,
                                    float scale_x = 10.0f,
                                    float scale_y = 10.0f,
                                    float scale_z = 1.5f,
                                    float scale_heading = 0.25f) const;


    /**
     * \brief   Compute velocity command from the manual input
     *
     * \param   command                     Velocity command (output)
     * \param   current_attitude            Current attitude of the vehicle
     * \param   current_velocity_command    Current velocity command
     * \param   min_vel         Minimum norm of velocity (when throttle stick is low)
     * \param   max_vel         Maximum norm of velocity (when throttle stick is high)
     * \param   scale_pitch     Scale (maximum output / max remote input)
     */
    void get_velocity_command_wing(velocity_command_t& command,
                                    const quat_t& current_attitude,
                                    const velocity_command_t& current_velocity_command,
                                    float min_vel = 0.0f,
                                    float max_vel = 20.0f,
                                    float scale_pitch = 1.0f,
                                    float scale_heading = 0.25f) const;


    /**
     * \brief   Returns the value of the mode from the desired source input
     *
     * \param   mode_current            The current mode of the MAV
     *
     * \return  The value of the mode
     */
    Mav_mode get_mode_from_source(Mav_mode mode_current);


    /**
     * \brief   Sets the internal state of the joystick
     *
     * \details only mode of joystick is overwritten, remote keeps its interal state
     *
     * \param   mode_current            The current mode of the MAV
     *
     */
    void set_mode_of_source(Mav_mode mode_current);

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
    Joystick                joystick;           ///< The pointer to the joystick structure
    mode_source_t           mode_source_;        ///< The source mode
    control_source_t        control_source_;     ///< Flag to tell whether the remote is active or not
};

Manual_control::conf_t Manual_control::default_config()
{
    conf_t conf = {};

    conf.joystick_config = Joystick::default_config();
    conf.mode_source = MODE_SOURCE_REMOTE;
    conf.control_source = CONTROL_SOURCE_REMOTE;

    return conf;
};





#endif /* MANUAL_CONTROL_HPP_ */
