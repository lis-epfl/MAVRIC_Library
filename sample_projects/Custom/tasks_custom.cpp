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
 * \file tasks.c
 *
 * \author MAV'RIC Team
 *
 * \brief Definition of the tasks executed on the autopilot
 *
 ******************************************************************************/


#include "sample_projects/LEQuad/tasks.hpp"
#include "sample_projects/Custom/tasks_custom.hpp"
#include "central_data_custom.hpp"


#include "communication/data_logging.hpp"

bool tasks_run_stabilisation_quaternion(Central_data_custom* central_data);
bool tasks_run_stabilisation_quaternion(Central_data_custom* central_data)
{
    tasks_run_imu_update(central_data);

    const State& state = central_data->state;

    if (state.is_armed() == false)
    {
        // Set command to current heading
        central_data->command.attitude.rpy[2] = coord_conventions_quat_to_aero(central_data->ahrs.qe).rpy[2];
        central_data->servo_0.failsafe();
        central_data->servo_1.failsafe();
        central_data->servo_2.failsafe();
        central_data->servo_3.failsafe();

        central_data->saccade_controller_.is_time_initialized_ = false;
    }
    else if (state.is_auto())
    {
        central_data->command.attitude = central_data->saccade_controller_.attitude_command_;

        // 1m altitude command (Above goround level)
        central_data->command.position.xyz[0] = 0.0f;
        central_data->command.position.xyz[1] = 0.0f;
        central_data->command.position.xyz[2] = -0.7f;
        central_data->command.position.mode   = POSITION_COMMAND_MODE_LOCAL;

        // Do control
        central_data->altitude_controller_.update();
        attitude_controller_update(&central_data->attitude_controller);

        servos_mix_quadcopter_diag_update(&central_data->servo_mix);

    }
    else if (state.is_manual() && state.is_guided())
    {
        // manual_control_get_velocity_command(&central_data->manual_control, &central_data->command.velocity, 1.0f);
        // velocity_controller_copter_update(&central_data->velocity_controller);

        // get attitude command from remote
        central_data->manual_control.get_attitude_command(0.02f, &central_data->command.attitude, 1.0f);

        // 1m altitude command (Above goround level)
        central_data->command.position.xyz[0] = 0.0f;
        central_data->command.position.xyz[1] = 0.0f;
        central_data->command.position.xyz[2] = -0.7f;
        central_data->command.position.mode   = POSITION_COMMAND_MODE_LOCAL;

        // Do control
        central_data->altitude_controller_.update();
        attitude_controller_update(&central_data->attitude_controller);

        servos_mix_quadcopter_diag_update(&central_data->servo_mix);

        central_data->saccade_controller_.saccade_state_ = PRESACCADE;

        central_data->saccade_controller_.is_time_initialized_ = false;
    }
    else if (state.is_manual() && state.is_stabilize())
    {

        // get command from remote
        central_data->manual_control.get_attitude_command(0.02f, &central_data->command.attitude, 1.0f);
        central_data->manual_control.get_thrust_command(&central_data->command.thrust);

        // Do control
        attitude_controller_update(&central_data->attitude_controller);

        servos_mix_quadcopter_diag_update(&central_data->servo_mix);

        // // get command from remote
        // manual_control_get_rate_command(&central_data->manual_control, &central_data->command.rate, 3.0f);
        // manual_control_get_thrust_command(&central_data->manual_control, &central_data->command.thrust);

        // // Do control
        // central_data->attitude_controller.mode = ATTITUDE_CONTROLLER_MODE_RATE_ONLY;
        // attitude_controller_update(&central_data->attitude_controller);

        // servos_mix_quadcopter_diag_update(&central_data->servo_mix);

        central_data->saccade_controller_.saccade_state_ = PRESACCADE;

        central_data->saccade_controller_.is_time_initialized_ = false;
    }
    else
    {
        central_data->servo_0.failsafe();
        central_data->servo_1.failsafe();
        central_data->servo_2.failsafe();
        central_data->servo_3.failsafe();
    }

    return true;
}


bool tasks_flow(Central_data_custom* cd)
{
    bool success = true;

    success &= cd->flow_left_.update();
    success &= cd->flow_right_.update();

    return success;
}


bool tasks_saccade(Central_data_custom* cd)
{
  cd->saccade_controller_.update();

  return true;
}


bool tasks_create_tasks(Central_data_custom* central_data)
{
    bool init_success = true;

    Scheduler* scheduler = &central_data->scheduler;

    // init_success &= scheduler->add_task(4000,     Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::Scheduler_task::PRIORITY_HIGHEST, (Scheduler_task::task_function_t)&tasks_run_stabilisation                         , (Scheduler_task::task_argument_t)central_data                         , 0);
    init_success &= scheduler->add_task(4000,     Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::Scheduler_task::PRIORITY_HIGHEST, (Scheduler_task::task_function_t)&tasks_run_stabilisation_quaternion              , (Scheduler_task::task_argument_t)central_data                         , 0);
    init_success &= scheduler->add_task(500000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_LOW    , (Scheduler_task::task_function_t)&tasks_led_toggle                                , (Scheduler_task::task_argument_t)&central_data->led                   , 1);

    init_success &= scheduler->add_task(15000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_RELATIVE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&tasks_run_barometer_update                      , (Scheduler_task::task_argument_t)central_data                     , 2);
    init_success &= scheduler->add_task(100000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&tasks_run_gps_update                            , (Scheduler_task::task_argument_t)central_data                     , 3);
    init_success &= scheduler->add_task(10000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&Navigation::update                               , (Scheduler_task::task_argument_t)&central_data->navigation            , 5);
    init_success &= scheduler->add_task(10000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&Mavlink_waypoint_handler::update                 , (Scheduler_task::task_argument_t)&central_data->waypoint_handler      , 6);
    init_success &= scheduler->add_task(200000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL , (Scheduler_task::task_function_t)&State_machine::update                            , (Scheduler_task::task_argument_t)&central_data->state_machine         , 7);
    init_success &= scheduler->add_task(4000,     Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL , (Scheduler_task::task_function_t)&Mavlink_communication::update                    , (Scheduler_task::task_argument_t)&central_data->mavlink_communication , 8);
    init_success &= scheduler->add_task(20000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH , (Scheduler_task::task_function_t)&remote_update                                     , (Scheduler_task::task_argument_t)&central_data->manual_control.remote , 10);
    // init_success &= scheduler->add_task(4000,      Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_LOWEST , (Scheduler_task::task_function_t)&tasks_sleep                                     , (Scheduler_task::task_argument_t)central_data                         , 14);
    init_success &= scheduler->add_task(10000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL , (Scheduler_task::task_function_t)&tasks_data_logging_update                       , (Scheduler_task::task_argument_t)central_data                         , 11);

    init_success &= scheduler->add_task(100000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH    , (Scheduler_task::task_function_t)&tasks_run_sonar_update                         , (Scheduler_task::task_argument_t)central_data                         , 12);
    init_success &= scheduler->add_task(4000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL,   (Scheduler_task::task_function_t)&tasks_altitude_estimation, (Scheduler_task::task_argument_t)&central_data->altitude_estimation_,   13);
    init_success &= scheduler->add_task(4000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Scheduler_task::task_function_t)&tasks_flow,                (Scheduler_task::task_argument_t)central_data,                          14);
    init_success &= scheduler->add_task(10000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Scheduler_task::task_function_t)&tasks_saccade,             (Scheduler_task::task_argument_t)central_data,                       15);
    // init_success &= scheduler_add_task(scheduler, 4000,      RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_LOWEST , (task_function_t)&tasks_sleep                                     , (task_argument_t)central_data                         , 99);

    init_success &= scheduler->sort_tasks();

    return init_success;
}
