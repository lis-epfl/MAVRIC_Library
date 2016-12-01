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
 * \file lequad.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#include "drones/lequad.hpp"

#include "communication/data_logging_telemetry.hpp"
#include "status/state_telemetry.hpp"

#include "drivers/barometer_telemetry.hpp"
#include "drivers/gps_telemetry.hpp"
#include "drivers/sonar_telemetry.hpp"
#include "drivers/px4flow_telemetry.hpp"

#include "sensing/imu_telemetry.hpp"
#include "sensing/ins_telemetry.hpp"
#include "sensing/ahrs_telemetry.hpp"

#include "manual_control/manual_control_telemetry.hpp"

#include "runtime/scheduler_telemetry.hpp"

#include "hal/common/time_keeper.hpp"

#include "util/print_util.hpp"


LEQuad::LEQuad(Imu& imu,
               Barometer& barometer,
               Gps& gps,
               Sonar& sonar,
               Px4flow_i2c& flow,
               Serial& serial_mavlink,
               Satellite& satellite,
               State_display& state_display,
               File& file_flash,
               Battery& battery,
               File& file1,
               File& file2,
               Servo& servo_0,
               Servo& servo_1,
               Servo& servo_2,
               Servo& servo_3,
               const conf_t& config):
    MAV(imu, barometer, gps, sonar, flow, serial_mavlink, satellite, state_display, file_flash, battery, file1, file2, flight_controller_quadcopter_, config.mav_config),
    flight_controller_quadcopter_(ins_, ahrs_, servo_0, servo_1, servo_2, servo_3, config.flight_controller_config)
{}


bool LEQuad::init_controller(void)
{
    bool ret = true;

    // Parameters
    Onboard_parameters& op = communication.parameters();
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_X().p_gain,               "C_RAT_X_KP");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_X().integrator.clip_pre,  "C_RAT_X_I_CLPRE");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_X().integrator.gain,      "C_RAT_X_KI");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_X().integrator.clip,      "C_RAT_X_I_CLIP");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_X().differentiator.gain,  "C_RAT_X_KD");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_Y().p_gain,               "C_RAT_Y_KP");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_Y().integrator.clip_pre,  "C_RAT_Y_I_CLPRE");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_Y().integrator.gain,      "C_RAT_Y_KI");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_Y().integrator.clip,      "C_RAT_Y_I_CLIP");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_Y().differentiator.gain,  "C_RAT_Y_KD");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_Z().p_gain,               "C_RAT_Z_KP");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_Z().integrator.clip_pre,  "C_RAT_Z_I_CLPRE");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_Z().integrator.gain,      "C_RAT_Z_KI");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_Z().integrator.clip,      "C_RAT_Z_I_CLIP");
    ret &= op.add(&flight_controller_quadcopter_.rate_ctrl_.get_pid_Z().differentiator.gain,  "C_RAT_Z_KD");

    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_X().p_gain,               "C_ATT_X_KP");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_X().integrator.clip_pre,  "C_ATT_X_I_CLPRE");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_X().integrator.gain,      "C_ATT_X_KI");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_X().integrator.clip,      "C_ATT_X_I_CLIP");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_X().differentiator.gain,  "C_ATT_X_KD");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_Y().p_gain,               "C_ATT_Y_KP");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_Y().integrator.clip_pre,  "C_ATT_Y_I_CLPRE");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_Y().integrator.gain,      "C_ATT_Y_KI");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_Y().integrator.clip,      "C_ATT_Y_I_CLIP");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_Y().differentiator.gain,  "C_ATT_Y_KD");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_Z().p_gain,               "C_ATT_Z_KP");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_Z().integrator.clip_pre,  "C_ATT_Z_I_CLPRE");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_Z().integrator.gain,      "C_ATT_Z_KI");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_Z().integrator.clip,      "C_ATT_Z_I_CLIP");
    ret &= op.add(&flight_controller_quadcopter_.att_ctrl_.get_pid_Z().differentiator.gain,  "C_ATT_Z_KD");

    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_X().p_gain,               "C_VEL_X_KP");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_X().integrator.clip_pre,  "C_VEL_X_I_CLPRE");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_X().integrator.gain,      "C_VEL_X_KI");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_X().integrator.clip,      "C_VEL_X_I_CLIP");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_X().differentiator.gain,  "C_VEL_X_KD");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_Y().p_gain,               "C_VEL_Y_KP");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_Y().integrator.clip_pre,  "C_VEL_Y_I_CLPRE");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_Y().integrator.gain,      "C_VEL_Y_KI");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_Y().integrator.clip,      "C_VEL_Y_I_CLIP");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_Y().differentiator.gain,  "C_VEL_Y_KD");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_Z().p_gain,               "C_VEL_Z_KP");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_Z().integrator.clip_pre,  "C_VEL_Z_I_CLPRE");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_Z().integrator.gain,      "C_VEL_Z_KI");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_Z().integrator.clip,      "C_VEL_Z_I_CLIP");
    ret &= op.add(&flight_controller_quadcopter_.vel_ctrl_.get_pid_Z().differentiator.gain,  "C_VEL_Z_KD");

    return ret;
}
