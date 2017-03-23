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
 * \file flight_controller_fixedwing.hxx
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Full flight controller for fixed wings
 *
 ******************************************************************************/

 #ifndef FLIGHT_CONTROLLER_FIXEDWINGS_HXX_
 #define FLIGHT_CONTROLLER_FIXEDWINGS_HXX_

template<uint32_t N_ROTORS>
Flight_controller_fixedwing<N_ROTORS>::Flight_controller_fixedwing(const INS& ins, const AHRS& ahrs, typename Servos_mix_matrix<N_ROTORS>::args_t mix_args, conf_t config):
    Flight_controller_stack(pos_ctrl_, vel_ctrl_, att_ctrl_, rate_ctrl_, mix_ctrl_),
    pos_ctrl_({ahrs, ins}, config.pos_config),
    vel_ctrl_({{ahrs, ins, command_.velocity, command_.attitude, command_.thrust}}, config.vel_config),
    att_ctrl_({ahrs, command_.attitude, command_.rate}, config.att_config),
    rate_ctrl_({ahrs, command_.rate, command_.torque}, config.rate_config),
    mix_ctrl_(mix_args, config.mix_config),
    ahrs_(ahrs)
{};


template<uint32_t N_ROTORS>
bool Flight_controller_fixedwing<N_ROTORS>::set_manual_rate_command(const Manual_control& manual_control)
{
    bool ret = true;
    rate_command_t rate_command;
    thrust_command_t thrust_command;
    manual_control.get_rate_command(rate_command, 1.0f, 1.0f, 0.0f);
    manual_control.get_thrust_command_wing(thrust_command);
    ret &= set_command(rate_command);
    ret &= set_command(thrust_command);
    return ret;
};


template<uint32_t N_ROTORS>
bool Flight_controller_fixedwing<N_ROTORS>::set_manual_attitude_command(const Manual_control& manual_control)
{
    bool ret = true;
    attitude_command_t att_command;
    thrust_command_t thrust_command;
    manual_control.get_attitude_command(att_command, ahrs_.attitude(), 1.0f, 1.0f, 0.0f);
    manual_control.get_thrust_command_wing(thrust_command);
    ret &= set_command(att_command);
    ret &= set_command(thrust_command);
    return ret;
};


template<uint32_t N_ROTORS>
bool Flight_controller_fixedwing<N_ROTORS>::set_manual_velocity_command(const Manual_control& manual_control)
{
    bool ret = true;
    velocity_command_t new_vel_command;
    manual_control.get_velocity_command_wing(new_vel_command, ahrs_.attitude(), command_.velocity);
    ret &= set_command(new_vel_command);
    return ret;
};

#endif  // FLIGHT_CONTROLLER_FIXEDWINGS_HXX_
