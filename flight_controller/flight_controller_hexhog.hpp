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
 * \file flight_controller_hexhog.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Flight controller for the hexhog
 *
 ******************************************************************************/

#ifndef FLIGHT_CONTROLLER_HEXHOG_HPP_
#define FLIGHT_CONTROLLER_HEXHOG_HPP_

#include "flight_controller/flight_controller_stack.hpp"
#include "control/position_controller.hpp"
#include "control/velocity_controller_holonomic.hpp"
#include "control/attitude_controller.hpp"
#include "control/rate_controller.hpp"
#include "control/servos_mix_matrix.hpp"

/**
 * \brief   Flight controller for the HexHog
 */
class Flight_controller_hexhog: public Flight_controller_stack
{
public:
    /**
     * \brief Configuration structure
     */
    struct conf_t
    {
        Position_controller::conf_t            pos_config;
        Velocity_controller_holonomic::conf_t  vel_config;
        Attitude_controller::conf_t            att_config;
        Rate_controller::conf_t                rate_config;
        Servos_mix_matrix<6>::conf_t           mix_config;
    };

    /**
     * \brief   Default Configuration
     *
     * /return  config
     */
    static conf_t default_config(void)
    {
        conf_t conf;

        conf.pos_config  = Position_controller::default_config();
        conf.vel_config  = Velocity_controller_holonomic::default_config();
        conf.att_config  = Attitude_controller::default_config();
        conf.rate_config = Rate_controller::default_config();

        float s60 = 0.866025f;
        conf.mix_config  = Servos_mix_matrix<6>::default_config();
        conf.mix_config.mix  = Mat<6, 6>({ 0.0f, -1.0f,  1.0f, -1.0f, 0.0f, -1.0f,   // rear
                                            s60, -0.5f, -1.0f,  0.5f,  s60, -1.0f,   // rear left
                                            s60,  0.5f,  1.0f,  0.5f, -s60, -1.0f,   // front left
                                           0.0f,  1.0f, -1.0f, -1.0f, 0.0f, -1.0f,   // front
                                           -s60,  0.5f,  1.0f,  0.5f,  s60, -1.0f,   // front right
                                           -s60, -0.5f, -1.0f,  0.5f, -s60, -1.0f}); // rear right
        return conf;
    };

    /**
     * \brief   Constructor
     */
    Flight_controller_hexhog(const INS& ins,
                            const AHRS& ahrs,
                            Servo& motor_rear,
                            Servo& motor_rear_left,
                            Servo& motor_front_left,
                            Servo& motor_front,
                            Servo& motor_front_right,
                            Servo& motor_rear_right,
                            conf_t config = default_config());

    /**
     * \brief   Set command from manual control
     *
     * \param   manual_control  Reference to manual_control
     */
    bool set_manual_command(const Manual_control& manual_control);

    /**
     * \brief   Set command from manual control in rate mode
     *
     * \param   manual_control  Reference to manual_control
     */
    bool set_manual_rate_command(const Manual_control& manual_control);

    /**
     * \brief   Set command from manual control in attitude mode
     *
     * \param   manual_control  Reference to manual_control
     */
    bool set_manual_attitude_command(const Manual_control& manual_control);

    /**
     * \brief   Set command from manual control in velocity mode
     *
     * \param   manual_control  Reference to manual_control
     */
    bool set_manual_velocity_command(const Manual_control& manual_control);

public:
    Position_controller             pos_ctrl_;  ///< Position controller
    Velocity_controller_holonomic   vel_ctrl_;  ///< Velocity controller
    Attitude_controller             att_ctrl_;  ///< Attitude controller
    Rate_controller                 rate_ctrl_; ///< Rate controller
    Servos_mix_matrix<6>            mix_ctrl_;  ///< Servos mix

private:
    const AHRS& ahrs_;                          ///< Reference to estimated attitude
};

#endif  // FLIGHT_CONTROLLER_HEXHOG_HPP_
