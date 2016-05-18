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
#include "sample_projects/LEQuad/lequad.hpp"
#include "communication/data_logging.hpp"
#include "hal/common/time_keeper.hpp"

void tasks_run_imu_update(LEQuad* mav)
{
    mav->imu.update();
    //qfilter_update(&mav->attitude_filter);
    mav->ahrs_ekf.update();
    mav->position_estimation.update();
}

bool tasks_run_stabilisation(LEQuad* mav)
{
    tasks_run_imu_update(mav);

    const State& state = mav->state;

    if (state.is_armed())
    {
        if (state.is_auto())
        {
            mav->controls = mav->controls_nav;
            mav->controls.control_mode = VELOCITY_COMMAND_MODE;

            // if no waypoints are set, we do position hold therefore the yaw mode is absolute
            if (((mav->state.nav_plan_active && (mav->navigation.internal_state_ == Navigation::NAV_NAVIGATING)) || (mav->navigation.internal_state_ == Navigation::NAV_STOP_THERE)) || ((mav->state.mav_state_ == MAV_STATE_CRITICAL) && (mav->navigation.critical_behavior == Navigation::FLY_TO_HOME_WP)))
            {
                mav->controls.yaw_mode = YAW_RELATIVE;
            }
            else
            {
                mav->controls.yaw_mode = YAW_ABSOLUTE;
            }

            //if (mav->state.in_the_air || mav->navigation.auto_takeoff)
            if (true)//mav->navigation.internal_state_ > NAV_ON_GND)
            {
                stabilisation_copter_cascade_stabilise(&mav->stabilisation_copter);
                servos_mix_quadcopter_diag_update(&mav->servo_mix);
            }
        }
        else if (state.is_guided())
        {
            mav->controls = mav->controls_nav;
            mav->controls.control_mode = VELOCITY_COMMAND_MODE;

            if ( ((mav->state.mav_state_ == MAV_STATE_CRITICAL) && (mav->navigation.critical_behavior == Navigation::FLY_TO_HOME_WP))  || (mav->navigation.navigation_strategy == Navigation::strategy_t::DUBIN))
            {
                mav->controls.yaw_mode = YAW_RELATIVE;
            }
            else
            {
                mav->controls.yaw_mode = YAW_ABSOLUTE;
            }

            //if (mav->state.in_the_air || mav->navigation.auto_takeoff)
            if (true)//mav->navigation.internal_state_ > NAV_ON_GND)
            {
                stabilisation_copter_cascade_stabilise(&mav->stabilisation_copter);
                servos_mix_quadcopter_diag_update(&mav->servo_mix);
            }
        }
        else if (state.is_stabilize())
        {
            mav->manual_control.get_velocity_vector(&mav->controls);

            mav->controls.control_mode = VELOCITY_COMMAND_MODE;
            mav->controls.yaw_mode = YAW_RELATIVE;

            //if (mav->state.in_the_air || mav->navigation.auto_takeoff)
            if (true)//mav->navigation.internal_state_ > NAV_ON_GND)
            {
                stabilisation_copter_cascade_stabilise(&mav->stabilisation_copter);
                servos_mix_quadcopter_diag_update(&mav->servo_mix);
            }
        }
        else if (state.is_manual())
        {
            mav->manual_control.get_control_command(&mav->controls);

            mav->controls.control_mode = ATTITUDE_COMMAND_MODE;
            mav->controls.yaw_mode = YAW_RELATIVE;

            stabilisation_copter_cascade_stabilise(&mav->stabilisation_copter);
            servos_mix_quadcopter_diag_update(&mav->servo_mix);
        }
        else
        {
            mav->servo_0.failsafe();
            mav->servo_1.failsafe();
            mav->servo_2.failsafe();
            mav->servo_3.failsafe();
        }
    }
    else
    {
        mav->servo_0.failsafe();
        mav->servo_1.failsafe();
        mav->servo_2.failsafe();
        mav->servo_3.failsafe();
    }

    return true;
}


// bool tasks_run_stabilisation_quaternion(LEQuad* mav);
// bool tasks_run_stabilisation_quaternion(LEQuad* mav)
// {
//     tasks_run_imu_update(mav);
//
//     const State& state = mav->state;
//
//     if (!state.is_armed())
//     {
//         // Set command to current heading
//         mav->command.attitude.rpy[2] = coord_conventions_quat_to_aero(mav->ahrs.qe).rpy[2];
//         mav->servo_0.failsafe();
//         mav->servo_1.failsafe();
//         mav->servo_2.failsafe();
//         mav->servo_3.failsafe();
//     }
//     else if (state.is_auto())
//     {
//         // Get command from vector field
//         vector_field_waypoint_update(&mav->vector_field_waypoint);
//
//         // Do control
//         velocity_controller_copter_update(&mav->velocity_controller);
//         attitude_controller_update(&mav->attitude_controller);
//
//         // Write output
//         servos_mix_quadcopter_diag_update(&mav->servo_mix);
//     }
//     else if (state.is_manual() && state.is_guided())
//     {
//         // Get command from remote
//         mav->manual_control.get_attitude_command(0.02f, &mav->command.attitude, 1.0f);
//         mav->manual_control.get_velocity_command(&mav->command.velocity, 1.0f);
//
//         // Do control
//         velocity_controller_copter_update(&mav->velocity_controller);
//         attitude_controller_update(&mav->attitude_controller);
//
//         // Write output
//         servos_mix_quadcopter_diag_update(&mav->servo_mix);
//     }
//     else if (state.is_manual() && state.is_stabilize())
//     {
//         // Get command from remote
//         mav->manual_control.get_attitude_command(0.02f, &mav->command.attitude, 1.0f);
//         mav->manual_control.get_thrust_command(&mav->command.thrust);
//
//         // Do control
//         attitude_controller_update(&mav->attitude_controller);
//
//         // Write output
//         servos_mix_quadcopter_diag_update(&mav->servo_mix);
//     }
//     else
//     {
//         mav->servo_0.failsafe();
//         mav->servo_1.failsafe();
//         mav->servo_2.failsafe();
//         mav->servo_3.failsafe();
//     }
//
//     return true;
// }



bool tasks_create_tasks(LEQuad* mav)
{
    bool init_success = true;

    Scheduler* scheduler = &mav->scheduler;

    init_success &= scheduler->add_task(4000,     Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::Scheduler_task::PRIORITY_HIGHEST, (Scheduler_task::task_function_t)&tasks_run_stabilisation                         , (Scheduler_task::task_argument_t)mav                         , 0);
    // init_success &= scheduler->add_task(4000,      Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::Scheduler_task::PRIORITY_HIGHEST, (Scheduler_task::task_function_t)&tasks_run_stabilisation_quaternion              , (Scheduler_task::task_argument_t)mav                         , 0);

    init_success &= scheduler->add_task(500000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_LOW    , (Scheduler_task::task_function_t)&task_led_toggle                                , (Scheduler_task::task_argument_t)&mav->led                   , 1);

    init_success &= scheduler->add_task(15000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_RELATIVE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&task_barometer_update                      , (Scheduler_task::task_argument_t)&mav->barometer                     , 2);
    init_success &= scheduler->add_task(100000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&task_gps_update                            , (Scheduler_task::task_argument_t)&mav->gps                     , 3);

    init_success &= scheduler->add_task(10000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&Navigation::update                               , (Scheduler_task::task_argument_t)&mav->navigation            , 5);
    init_success &= scheduler->add_task(10000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH   , (Scheduler_task::task_function_t)&Mavlink_waypoint_handler::update                 , (Scheduler_task::task_argument_t)&mav->waypoint_handler      , 6);

    init_success &= scheduler->add_task(200000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL , (Scheduler_task::task_function_t)&State_machine::update                            , (Scheduler_task::task_argument_t)&mav->state_machine         , 7);

    init_success &= scheduler->add_task(4000,     Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL , (Scheduler_task::task_function_t)&Mavlink_communication::update                    , (Scheduler_task::task_argument_t)&mav->mavlink_communication , 8);

    init_success &= scheduler->add_task(20000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH , (Scheduler_task::task_function_t)&remote_update                                     , (Scheduler_task::task_argument_t)&mav->manual_control.remote , 10);



    init_success &= scheduler->add_task(10000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL , (Scheduler_task::task_function_t)&task_data_logging_update                       , (Scheduler_task::task_argument_t)&mav->data_logging_continuous                         , 11);
    init_success &= scheduler->add_task(10000,    Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL , (Scheduler_task::task_function_t)&task_data_logging_update                       , (Scheduler_task::task_argument_t)&mav->data_logging_stat                         , 12);

    init_success &= scheduler->add_task(100000,   Scheduler_task::RUN_REGULAR, Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_HIGH    , (Scheduler_task::task_function_t)&task_sonar_update                         , (Scheduler_task::task_argument_t)&mav->sonar                         , 13);

    init_success &= scheduler->sort_tasks();

    return init_success;
}
