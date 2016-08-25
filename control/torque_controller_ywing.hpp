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
 * \file torque_controller_ywing.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Basil Huber
 *
 * \brief Links between torque commands and servos PWM command for Ywing
 *
 ******************************************************************************/


#ifndef TORQUE_CONTROLLER_YWING_HPP_
#define TORQUE_CONTROLLER_YWING_HPP_

#include "control/torque_controller.hpp"

class Torque_controller_ywing: class torque_controller
{
public:

  /**
   * \brief The servo mix structure for a Ywing
   */
  struct conf_t
  {
      flap_dir_t  flap_top_dir;       ///< Left  flap turning direction
      flap_dir_t  flap_right_dir;     ///< Right flap turning direction
      flap_dir_t  flap_left_dir;      ///< Rear  flap turning direction
      float       min_thrust;         ///< Minimum thrust
      float       max_thrust;         ///< Maximum thrust
      float       min_deflection;     ///< Minimum deflection for flaps
      float       max_deflection;     ///< Maximum deflection for flaps
  };


  /**
   * \brief Constructor arguments
   */
  struct args_t
  {
      Servo& servo_motor;
      Servo& servo_flap_top;
      Servo& servo_flap_right;
      Servo& servo_flap_left;
  };

  Torque_controller_ywing(args_t& args, const conf_t& config = default_config());

  virtual void update();


  static inline conf_t default_config();

private:
    flap_dir_t  flap_top_dir_;       ///< Left  flap turning direction
    flap_dir_t  flap_right_dir_;     ///< Right flap turning direction
    flap_dir_t  flap_left_dir_;      ///< Rear  flap turning direction
    float       min_thrust_;         ///< Minimum thrust
    float       max_thrust_;         ///< Maximum thrust
    float       min_deflection_;     ///< Minimum deflection for flaps
    float       max_deflection_;     ///< Maximum deflection for flaps
    Servo&      motor_;               ///< Servos structure for main motor
    Servo&      flap_top_;            ///< Servos structure for top flap
    Servo&      flap_right_;          ///< Servos structure for right flap
    Servo&      flap_left_;           ///< Servos structure for left flap
};



Torque_controller::conf_t Torque_controller::default_config(){
{
  conf_t config;
    config.flap_top_dir   = FLAP_INVERTED,
    config.flap_right_dir = FLAP_INVERTED,
    config.flap_left_dir  = FLAP_INVERTED,
    config.min_thrust     = -0.9f,
    config.max_thrust     = 1.0f,
    config .min_deflection = -1.0f,
    config .max_deflection = 1.0f,
};

#endif
