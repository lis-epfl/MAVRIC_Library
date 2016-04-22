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
#include "sample_projects/LEQuad/central_data.hpp"
#include "communication/data_logging.hpp"


void tasks_run_imu_update(Central_data* central_data)
{
    central_data->imu.update();
    qfilter_update(&central_data->attitude_filter);
    central_data->position_estimation.update();
}

bool tasks_run_stabilisation(Central_data* central_data)
{
    tasks_run_imu_update(central_data);

    mav_mode_t mode = central_data->state.mav_mode();

    if (mav_modes_is_armed(mode))
    {
        if (mav_modes_is_auto(mode))
        {
            central_data->controls = central_data->controls_nav;
            central_data->controls.control_mode = VELOCITY_COMMAND_MODE;

            // if no waypoints are set, we do position hold therefore the yaw mode is absolute
            if (((central_data->state.nav_plan_active && (central_data->navigation.internal_state_ == Navigation::NAV_NAVIGATING)) || (central_data->navigation.internal_state_ == Navigation::NAV_STOP_THERE)) || ((central_data->state.mav_state_ == MAV_STATE_CRITICAL) && (central_data->navigation.critical_behavior == Navigation::FLY_TO_HOME_WP)))
            {
                central_data->controls.yaw_mode = YAW_RELATIVE;
            }
            else
            {
                central_data->controls.yaw_mode = YAW_ABSOLUTE;
            }

            //if (central_data->state.in_the_air || central_data->navigation.auto_takeoff)
            if (true)//central_data->navigation.internal_state_ > NAV_ON_GND)
            {
                stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);
                servos_mix_quadcopter_diag_update(&central_data->servo_mix);
            }
        }
        else if (mav_modes_is_guided(mode))
        {
            central_data->controls = central_data->controls_nav;
            central_data->controls.control_mode = VELOCITY_COMMAND_MODE;

            if ((central_data->state.mav_state_ == MAV_STATE_CRITICAL) && (central_data->navigation.critical_behavior == Navigation::FLY_TO_HOME_WP))
            {
                central_data->controls.yaw_mode = YAW_RELATIVE;
            }
            else
            {
                central_data->controls.yaw_mode = YAW_ABSOLUTE;
            }

            //if (central_data->state.in_the_air || central_data->navigation.auto_takeoff)
            if (true)//central_data->navigation.internal_state_ > NAV_ON_GND)
            {
                stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);
                servos_mix_quadcopter_diag_update(&central_data->servo_mix);
            }
        }
        else if (mav_modes_is_stabilise(mode))
        {
            central_data->manual_control.get_velocity_vector(&central_data->controls);

            central_data->controls.control_mode = VELOCITY_COMMAND_MODE;
            central_data->controls.yaw_mode = YAW_RELATIVE;

            //if (central_data->state.in_the_air || central_data->navigation.auto_takeoff)
            if (true)//central_data->navigation.internal_state_ > NAV_ON_GND)
            {
                stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);
                servos_mix_quadcopter_diag_update(&central_data->servo_mix);
            }
        }
        else if (mav_modes_is_manual(mode))
        {
            central_data->manual_control.get_control_command(&central_data->controls);

            central_data->controls.control_mode = ATTITUDE_COMMAND_MODE;
            central_data->controls.yaw_mode = YAW_RELATIVE;

            stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);
            servos_mix_quadcopter_diag_update(&central_data->servo_mix);
        }
        else
        {
            central_data->servo_0.failsafe();
            central_data->servo_1.failsafe();
            central_data->servo_2.failsafe();
            central_data->servo_3.failsafe();
        }
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


bool tasks_run_stabilisation_quaternion(Central_data* central_data);
bool tasks_run_stabilisation_quaternion(Central_data* central_data)
{
    tasks_run_imu_update(central_data);

    mav_mode_t mode = central_data->state.mav_mode();

    if (!central_data->state.armed())
    {
        // Set command to current heading
        central_data->command.attitude.rpy[2] = coord_conventions_quat_to_aero(central_data->ahrs.qe).rpy[2];
        central_data->servo_0.failsafe();
        central_data->servo_1.failsafe();
        central_data->servo_2.failsafe();
        central_data->servo_3.failsafe();
    }
    else if (mav_modes_is_auto(mode))
    {
        // vector_field_waypoint_update(&central_data->vector_field_waypoint);
        // velocity_controller_copter_update(&central_data->velocity_controller);
        // attitude_controller_update(&central_data->attitude_controller);
        // servos_mix_quadcopter_diag_update(&central_data->servo_mix);
    }
    else if (mav_modes_is_manual(mode) && mav_modes_is_guided(mode))
    {
        // manual_control_get_velocity_command(&central_data->manual_control, &central_data->command.velocity, 1.0f);
        // velocity_controller_copter_update(&central_data->velocity_controller);

        // get attitude command from remote
        central_data->manual_control.get_attitude_command(0.02f, &central_data->command.attitude, 1.0f);

        // Hardcode altitude command
        central_data->command.position.xyz[0] = 0.0f;
        central_data->command.position.xyz[1] = 0.0f;
        central_data->command.position.xyz[2] = -5.0f;
        central_data->command.position.mode   = POSITION_COMMAND_MODE_LOCAL;

        // Do control
        central_data->altitude_controller_.update();
        attitude_controller_update(&central_data->attitude_controller);

        servos_mix_quadcopter_diag_update(&central_data->servo_mix);
    }
    else if (mav_modes_is_manual(mode) && mav_modes_is_stabilise(mode))
    {
        // get command from remote
        central_data->manual_control.get_attitude_command(0.02f, &central_data->command.attitude, 1.0f);
        central_data->manual_control.get_thrust_command(&central_data->command.thrust);

        // Do control
        attitude_controller_update(&central_data->attitude_controller);

        servos_mix_quadcopter_diag_update(&central_data->servo_mix);
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


bool tasks_run_gps_update(Central_data* central_data)
{
    central_data->gps.update();

    return true;
}


bool tasks_run_barometer_update(Central_data* central_data)
{

    central_data->barometer.update();

    return true;
}


bool tasks_run_sonar_update(Central_data* central_data)
{
    central_data->sonar.update();

    return true;
}


bool tasks_sleep(Central_data* central_data)
{
    time_keeper_sleep_us(5000);
    return true;
}


bool tasks_led_toggle(Led* led)
{
    led->toggle();

    return true;
}

bool tasks_data_logging_update(Central_data* central_data)
{
    bool run_success = true;

    run_success &= central_data->data_logging.update();
    run_success &= central_data->data_logging2.update();

    return run_success;
}


bool tasks_altitude_estimation(Altitude_estimation* altitude_estimation)
{
    bool success = true;

    success &= altitude_estimation->update();

    return success;
}


bool tasks_create_tasks(Central_data* central_data)
{
    bool init_success = true;

    Scheduler* scheduler = &central_data->scheduler;

    init_success &= scheduler->add_task(4000,     Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::Scheduler_task::PRIORITY_HIGHEST, (Scheduler_task::task_function_t)&tasks_run_stabilisation                         , (Scheduler_task::task_argument_t)central_data                         , 0);
    // init_success &= scheduler->add_task(4000,      Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::Scheduler_task::PRIORITY_HIGHEST, (Scheduler_task::task_function_t)&tasks_run_stabilisation_quaternion              , (Scheduler_task::task_argument_t)central_data                         , 0);

    init_success &= scheduler->add_task(4000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH , (Scheduler_task::task_function_t)&tasks_altitude_estimation                          , (Scheduler_task::task_argument_t)&central_data->altitude_estimation_ , 12);

    init_success &= scheduler->add_task(15000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_RELATIVE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&tasks_run_barometer_update                      , (Scheduler_task::task_argument_t)central_data                     , 2);
    init_success &= scheduler->add_task(100000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&tasks_run_gps_update                            , (Scheduler_task::task_argument_t)central_data                     , 3);

    init_success &= scheduler->add_task(10000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&Navigation::update                               , (Scheduler_task::task_argument_t)&central_data->navigation            , 5);
    init_success &= scheduler->add_task(10000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&Mavlink_waypoint_handler::update                 , (Scheduler_task::task_argument_t)&central_data->waypoint_handler      , 6);

    init_success &= scheduler->add_task(200000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL , (Scheduler_task::task_function_t)&State_machine::update                            , (Scheduler_task::task_argument_t)&central_data->state_machine         , 7);

    init_success &= scheduler->add_task(4000,     Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL , (Scheduler_task::task_function_t)&Mavlink_communication::update                    , (Scheduler_task::task_argument_t)&central_data->mavlink_communication , 8);

    init_success &= scheduler->add_task(20000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH , (Scheduler_task::task_function_t)&remote_update                                     , (Scheduler_task::task_argument_t)&central_data->manual_control.remote , 10);

    init_success &= scheduler->add_task(100000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH    , (Scheduler_task::task_function_t)&tasks_run_sonar_update                         , (Scheduler_task::task_argument_t)central_data                         , 13);

    init_success &= scheduler->add_task(500000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_LOW    , (Scheduler_task::task_function_t)&tasks_led_toggle                                , (Scheduler_task::task_argument_t)&central_data->led                   , 1);

    // init_success &= scheduler->add_task(4000,      Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_LOWEST , (Scheduler_task::task_function_t)&tasks_sleep                                     , (Scheduler_task::task_argument_t)central_data                         , 14);

    init_success &= scheduler->add_task(10000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL , (Scheduler_task::task_function_t)&tasks_data_logging_update                       , (Scheduler_task::task_argument_t)central_data                         , 11);

    init_success &= scheduler->sort_tasks();

    return init_success;
}
