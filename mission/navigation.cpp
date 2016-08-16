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
 * \file navigation.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief Waypoint navigation controller
 *
 ******************************************************************************/


#include "mission/navigation.hpp"
#include "hal/common/time_keeper.hpp"
#include "control/dubin.hpp"
#include "mission/mission_handler.hpp"
#include "util/constants.hpp"

extern "C"
{
#include "util/print_util.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------
/**
 * \brief                   Computes the relative position and distance to the given way point
 *
 * \param   waypoint_pos        Local coordinates of the waypoint
 * \param   rel_pos         Array to store the relative 3D position of the waypoint
 * \param   local_pos       The 3D array of the actual position
 *
 * \return                  Distance to waypoint squared
 */
static float navigation_set_rel_pos_n_dist2wp(float waypoint_pos[], float rel_pos[], const float local_pos[3]);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static float navigation_set_rel_pos_n_dist2wp(float waypoint_pos[], float rel_pos[], const float local_pos[3])
{
    float dist2wp_sqr;

    rel_pos[X] = (float)(waypoint_pos[X] - local_pos[X]);
    rel_pos[Y] = (float)(waypoint_pos[Y] - local_pos[Y]);
    rel_pos[Z] = (float)(waypoint_pos[Z] - local_pos[Z]);

    dist2wp_sqr = vectors_norm_sqr(rel_pos);

    return dist2wp_sqr;
}


void Navigation::set_speed_command(float rel_pos[])
{
    float  norm_rel_dist;
    float v_desired = 0.0f;

    float dir_desired_sg[3];
    float rel_heading;

    Mav_mode mode = state.mav_mode();

    norm_rel_dist = sqrt(dist2wp_sqr);

    // calculate dir_desired in local frame
    aero_attitude_t attitude_yaw = coord_conventions_quat_to_aero(qe);
    attitude_yaw.rpy[0] = 0.0f;
    attitude_yaw.rpy[1] = 0.0f;
    attitude_yaw.rpy[2] = -attitude_yaw.rpy[2];
    quat_t q_rot = coord_conventions_quaternion_from_aero(attitude_yaw);

    quaternions_rotate_vector(q_rot, rel_pos, dir_desired_sg);

    // Avoiding division by zero
    if (norm_rel_dist < 0.0005f)
    {
        norm_rel_dist += 0.0005f;
    }

    // Normalisation of the goal direction
    dir_desired_sg[X] /= norm_rel_dist;
    dir_desired_sg[Y] /= norm_rel_dist;
    dir_desired_sg[Z] /= norm_rel_dist;

    // Check if we are hovering at a waypoint
    if ((mode.is_auto() && (!waiting_at_waypoint_)) || ((state.mav_state_ == MAV_STATE_CRITICAL) && (critical_behavior == Navigation::FLY_TO_HOME_WP)))
    {

        if (((maths_f_abs(rel_pos[X]) <= 1.0f) && (maths_f_abs(rel_pos[Y]) <= 1.0f)) || ((maths_f_abs(rel_pos[X]) <= 5.0f) && (maths_f_abs(rel_pos[Y]) <= 5.0f) && (maths_f_abs(rel_pos[Z]) >= 3.0f)))
        {
            rel_heading = 0.0f;
        }
        else
        {
            rel_heading = maths_calc_smaller_angle(atan2(rel_pos[Y], rel_pos[X]) - coord_conventions_get_yaw(qe));
        }

        wpt_nav_controller.clip_max = cruise_speed;
        v_desired = pid_controller_update_dt(&wpt_nav_controller, norm_rel_dist, dt);
    }
    else
    {
        rel_heading = 0.0f;
        hovering_controller.clip_max = cruise_speed;
        v_desired = pid_controller_update_dt(&hovering_controller, norm_rel_dist, dt);
    }

    if (v_desired *  maths_f_abs(dir_desired_sg[Z]) > max_climb_rate)
    {
        v_desired = max_climb_rate / maths_f_abs(dir_desired_sg[Z]);
    }


    // Scaling of the goal direction by the desired speed
    dir_desired_sg[X] *= v_desired;
    dir_desired_sg[Y] *= v_desired;
    dir_desired_sg[Z] *= v_desired;

    // loop_count++;
    // loop_count = loop_count % 50;
    // if (loop_count == 0)
    // {
    //  // print_util_dbg_print("Desired_vel_sg(x100): (");
    //  // print_util_dbg_print_num(dir_desired_sg[X] * 100,10);
    //  // print_util_dbg_print_num(dir_desired_sg[Y] * 100,10);
    //  // print_util_dbg_print_num(dir_desired_sg[Z] * 100,10);
    //  // print_util_dbg_print("). \r\n");
    //  print_util_dbg_print("rel_heading(x100): ");
    //  print_util_dbg_print_num(rel_heading,10);
    //  print_util_dbg_print("\r\n");
    //  print_util_dbg_print("nav state: ");
    //  print_util_dbg_print_num(internal_state_,10);
    //  print_util_dbg_print("\r\n");
    //  // print_util_dbg_print("Actual_vel_bf(x100): (");
    //  // print_util_dbg_print_num(position_estimation.vel_bf[X] * 100,10);
    //  // print_util_dbg_print_num(position_estimation.vel_bf[Y] * 100,10);
    //  // print_util_dbg_print_num(position_estimation.vel_bf[Z] * 100,10);
    //  // print_util_dbg_print("). \r\n");
    //  // print_util_dbg_print("Actual_pos(x100): (");
    //  // print_util_dbg_print_num(position_estimation.local_position.pos[X] * 100,10);
    //  // print_util_dbg_print_num(position_estimation.local_position.pos[Y] * 100,10);
    //  // print_util_dbg_print_num(position_estimation.local_position.pos[Z] * 100,10);
    //  // print_util_dbg_print("). \r\n");
    // }

    controls_nav.tvel[X] = dir_desired_sg[X];
    controls_nav.tvel[Y] = dir_desired_sg[Y];
    controls_nav.tvel[Z] = dir_desired_sg[Z];
    controls_nav.rpy[YAW] = kp_yaw * rel_heading;

    // Update handler control
    Mission_handler* handler = mission_handler_registry_.get_mission_handler(goal_);
    if (handler != NULL)
    {
        handler->modify_control_command(controls_nav);
    }
}

void Navigation::dubin_state_machine()
{
    float rel_pos[3];

    rel_pos[Z] = 0.0f;

    float dir_init_bf[3], dir_init[3], init_radius;

    float scalar_product, heading_diff;

    quat_t q_rot;
    aero_attitude_t attitude_yaw;
    local_position_t pos    = ins.position_lf();
    std::array<float,3> vel = ins.velocity_lf();

    float radius;
    if (!goal_.radius(radius))
    {
        radius = 0.0f;
    }
    switch(dubin_state)
    {
        case DUBIN_INIT:
            print_util_dbg_print("DUBIN_INIT\r\n");
            if (radius >= minimal_radius)
            {
                init_radius = maths_f_abs(radius);
            }
            else
            {
                init_radius = minimal_radius;
            }

            dir_init_bf[X] = init_radius;
            dir_init_bf[Y] = 0.0f;
            dir_init_bf[Z] = 0.0f;

            attitude_yaw = coord_conventions_quat_to_aero(qe);
            attitude_yaw.rpy[ROLL] = 0.0f;
            attitude_yaw.rpy[PITCH] = 0.0f;
            attitude_yaw.rpy[YAW] = attitude_yaw.rpy[2];
            q_rot = coord_conventions_quaternion_from_aero(attitude_yaw);

            quaternions_rotate_vector(q_rot, dir_init_bf, dir_init);

            for (uint8_t i = 0; i < 2; ++i)
            {
                rel_pos[i] = goal_.local_pos()[i]- pos[i];
            }
            rel_pos[Z] = 0.0f;

            float dir_final[3];
            float pos_goal[3];
            float rel_pos_norm[3];

            if (vectors_norm_sqr(rel_pos) > 0.1f)
            {
                vectors_normalize(rel_pos,rel_pos_norm);

                float end_radius;
                if (radius < minimal_radius)
                {
                    end_radius = minimal_radius;
                }
                else
                {
                    end_radius = radius;
                }

                dir_final[X] = -rel_pos_norm[Y]*end_radius;
                dir_final[Y] = rel_pos_norm[X]*end_radius;
                dir_final[Z] = 0.0f;

                for (uint8_t i = 0; i < 2; ++i)
                {
                    pos_goal[i] = goal_.local_pos()[i] + (rel_pos_norm[i] * maths_f_abs(radius));
                }
                pos_goal[Z] = 0.0f;

                goal_dubin_ = dubin_2d( pos.data(),
                                        pos_goal,
                                        dir_init,
                                        dir_final,
                                        maths_sign(end_radius));

                print_util_dbg_print("Entering DUBIN_CIRCLE_1\r\n");
                dubin_state = DUBIN_CIRCLE1;
            }
            else
            {
                dir_final[X] = -dir_init[Y];
                dir_final[Y] = dir_init[X];
                dir_final[Z] = 0.0f;

                for (uint8_t i = 0; i < 2; ++i)
                {
                    goal_dubin_.circle_center_2[i] = pos[i];
                }
                goal_dubin_.circle_center_2[Z] = 0.0f;

                print_util_dbg_print("Entering DUBIN_CIRCLE_2\r\n");
                dubin_state = DUBIN_CIRCLE2;
            }

            break;
        case DUBIN_CIRCLE1:
            for (uint8_t i = 0; i < 2; ++i)
            {
                rel_pos[i] = goal_dubin_.tangent_point_2[i] - pos[i];
            }
            heading_diff = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - atan2(vel[Y], vel[X]));

            if (maths_f_abs(heading_diff) < heading_acceptance)
            {
                print_util_dbg_print("Entering DUBIN_STRAIGHT\r\n");
                dubin_state = DUBIN_STRAIGHT;
            }
            break;
        case DUBIN_STRAIGHT:
            for (uint8_t i = 0; i < 2; ++i)
            {
                rel_pos[i] = goal_dubin_.tangent_point_2[i] - pos[i];
            }

            scalar_product = rel_pos[X] * goal_dubin_.line_direction[X] + rel_pos[Y] * goal_dubin_.line_direction[Y];
            if (scalar_product < 0.0f)
            {
                print_util_dbg_print("Entering DUBIN_CIRCLE_2\r\n");
                dubin_state = DUBIN_CIRCLE2;
            }

        case DUBIN_CIRCLE2:
        break;
    }
}

void Navigation::set_dubin_velocity(dubin_t* dubin)
{
    float dir_desired[3];

    quat_t q_rot;
    aero_attitude_t attitude_yaw;

    float radius;
    if (!goal_.radius(radius))
    {
        radius = 0.0f;
    }

    switch(dubin_state)
    {
        case DUBIN_INIT:
            dubin_circle(   dir_desired,
                            dubin->circle_center_1,
                            radius,
                            ins.position_lf().data(),
                            cruise_speed,
                            one_over_scaling );
            break;
        case DUBIN_CIRCLE1:
            dubin_circle(   dir_desired,
                            dubin->circle_center_1,
                            dubin->radius_1,
                            ins.position_lf().data(),
                            cruise_speed,
                            one_over_scaling );
        break;

        case DUBIN_STRAIGHT:
            dubin_line( dir_desired,
                        dubin->line_direction,
                        dubin->tangent_point_2,
                        ins.position_lf().data(),
                        cruise_speed,
                        one_over_scaling);
        break;

        case DUBIN_CIRCLE2:
            dubin_circle(   dir_desired,
                            dubin->circle_center_2,
                            radius,
                            ins.position_lf().data(),
                            cruise_speed,
                            one_over_scaling );
        break;
    }

    float vert_vel = vertical_vel_gain * (goal_.local_pos()[Z] - ins.position_lf()[Z]);

    if (maths_f_abs(vert_vel) > max_climb_rate)
    {
        vert_vel = maths_sign(vert_vel) * max_climb_rate;
    }

    dir_desired[Z] = vert_vel;

    // Transform the vector in the semi-global reference frame
    attitude_yaw = coord_conventions_quat_to_aero(qe);
    attitude_yaw.rpy[0] = 0.0f;
    attitude_yaw.rpy[1] = 0.0f;
    attitude_yaw.rpy[2] = -attitude_yaw.rpy[2];
    q_rot = coord_conventions_quaternion_from_aero(attitude_yaw);

    float dir_desired_sg[3];
    quaternions_rotate_vector(q_rot, dir_desired, dir_desired_sg);

    controls_nav.tvel[X] = dir_desired_sg[X];
    controls_nav.tvel[Y] = dir_desired_sg[Y];
    controls_nav.tvel[Z] = dir_desired_sg[Z];

    float rel_heading;
    rel_heading = maths_calc_smaller_angle(atan2(dir_desired[Y],dir_desired[X]) - coord_conventions_get_yaw(qe));

    controls_nav.rpy[YAW] = kp_yaw * rel_heading;

    // Update handler control
    Mission_handler* handler = mission_handler_registry_.get_mission_handler(goal_);
    if (handler != NULL)
    {
        handler->modify_control_command(controls_nav);
    }
}


void Navigation::run()
{
    float rel_pos[3];

    // Control in translational speed of the platform
    dist2wp_sqr = navigation_set_rel_pos_n_dist2wp( goal_.local_pos().data(),
                                                    rel_pos,
                                                    ins.position_lf().data());

    switch(navigation_strategy)
    {
        case Navigation::strategy_t::DIRECT_TO:
            set_speed_command(rel_pos);
        break;

        case Navigation::strategy_t::DUBIN:
            if (state.autopilot_type == MAV_TYPE_QUADROTOR)
            {
                //if (internal_state_ == NAV_NAVIGATING) TODO
                {
                    float radius;
                    if (!goal_.radius(radius))
                    {
                        radius = 0.0f;
                    }

                    if (radius > 0.0f)
                    {
                        set_dubin_velocity(&goal_dubin_);
                    }
                    else
                    {
                        set_speed_command(rel_pos);
                    }

                }
                /*else
                {
                    set_speed_command(rel_pos);
                }*/
            }
            else
            {
                set_dubin_velocity(&goal_dubin_);
            }
            // Add here other types of navigation strategies
        break;
    }

    float heading;
    if (!goal_.heading(heading))
    {
        heading = 0.0f;
    }
    controls_nav.theading = heading;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Navigation::Navigation(control_command_t& controls_nav, const quat_t& qe, const INS& ins, State& state, Mavlink_stream& mavlink_stream, Mission_handler_registry& mission_handler_registry, conf_t nav_config) :
    qe(qe),
    controls_nav(controls_nav),
    ins(ins),
    state(state),
    mavlink_stream(mavlink_stream),
    mission_handler_registry_(mission_handler_registry)
{
    //navigation controller init

    controls_nav.rpy[ROLL] = 0.0f;
    controls_nav.rpy[PITCH] = 0.0f;
    controls_nav.rpy[YAW] = 0.0f;
    controls_nav.tvel[X] = 0.0f;
    controls_nav.tvel[Y] = 0.0f;
    controls_nav.tvel[Z] = 0.0f;
    controls_nav.theading = 0.0f;
    controls_nav.thrust = -1.0f;
    controls_nav.control_mode = VELOCITY_COMMAND_MODE;
    controls_nav.yaw_mode = YAW_ABSOLUTE;

    goal_ = Waypoint();

    last_update = 0;

    dist2wp_sqr = 0.0f;
    waiting_at_waypoint_ = false;
    start_wpt_time_ = time_keeper_get_ms();

    wpt_nav_controller = nav_config.wpt_nav_controller;
    hovering_controller = nav_config.hovering_controller;

    one_over_scaling = nav_config.one_over_scaling;

    safe_altitude = nav_config.safe_altitude;
    critical_landing_altitude = nav_config.critical_landing_altitude;
    minimal_radius = nav_config.minimal_radius;
    heading_acceptance = nav_config.heading_acceptance;
    vertical_vel_gain = nav_config.vertical_vel_gain;
    takeoff_altitude = nav_config.takeoff_altitude;

    soft_zone_size = nav_config.soft_zone_size;

    dist2vel_gain = nav_config.dist2vel_gain;
    cruise_speed = nav_config.cruise_speed;
    max_climb_rate = nav_config.max_climb_rate;

    navigation_strategy = nav_config.navigation_strategy;
    dubin_state = DUBIN_INIT;

    critical_behavior = CLIMB_TO_SAFE_ALT;
    auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;

    alt_lpf = nav_config.alt_lpf;
    LPF_gain = nav_config.LPF_gain;
    kp_yaw = nav_config.kp_yaw;

    loop_count = 0;

    dt = 0.004;
}


bool Navigation::update(Navigation* navigation)
{
    Mav_mode mode_local = navigation->state.mav_mode();

    uint32_t t = time_keeper_get_us();

    navigation->dt = (float)(t - navigation->last_update) / 1000000.0f;
    navigation->last_update = t;

    // Update the dubin structure
    if (navigation->navigation_strategy == strategy_t::DUBIN)
    {
        navigation->dubin_state_machine();
    }

    switch (navigation->state.mav_state_)
    {
        case MAV_STATE_STANDBY:
            navigation->controls_nav.tvel[X] = 0.0f;
            navigation->controls_nav.tvel[Y] = 0.0f;
            navigation->controls_nav.tvel[Z] = 0.0f;
            break;

        case MAV_STATE_ACTIVE:
            if (navigation->goal_.command() != 0) // If we do not have the on ground waypoint set
            {
                navigation->run();
            }
            break;

        case MAV_STATE_CRITICAL:
            // In MAV_MODE_VELOCITY_CONTROL, MAV_MODE_POSITION_HOLD and MAV_MODE_GPS_NAVIGATION
            if (mode_local.is_guided())
            {
                navigation->run();

                if (navigation->state.out_of_fence_2)
                {
                    // Constant velocity to the ground
                    navigation->controls_nav.tvel[Z] = 1.0f;
                }
            }
            break;

        default:
            navigation->controls_nav.tvel[X] = 0.0f;
            navigation->controls_nav.tvel[Y] = 0.0f;
            navigation->controls_nav.tvel[Z] = 0.0f;
            break;
    }
    
    return true;
}

void Navigation::set_goal(Waypoint new_goal)
{
    // check for change in goal
    if (new_goal.command() != goal_.command() ||
        new_goal.frame() != goal_.frame() ||
        maths_f_abs(new_goal.param1() - goal_.param1()) > 0.000001f ||
        maths_f_abs(new_goal.param2() - goal_.param2()) > 0.000001f ||
        maths_f_abs(new_goal.param3() - goal_.param3()) > 0.000001f ||
        maths_f_abs(new_goal.param4() - goal_.param4()) > 0.000001f ||
        maths_f_abs(new_goal.param5() - goal_.param5()) > 0.000001f ||
        maths_f_abs(new_goal.param6() - goal_.param6()) > 0.000001f ||
        maths_f_abs(new_goal.param7() - goal_.param7()) > 0.000001f ||
        maths_f_abs(new_goal.local_pos()[X] - goal_.local_pos()[X]) > 0.000001f ||
        maths_f_abs(new_goal.local_pos()[Y] - goal_.local_pos()[Y]) > 0.000001f ||
        maths_f_abs(new_goal.local_pos()[Z] - goal_.local_pos()[Z]) > 0.000001f)
    {
        print_util_dbg_print("Waypoint changed\r\n");
        print_util_dbg_print("New goal: command: ");
        print_util_dbg_print_num(new_goal.command(), 10);
        print_util_dbg_print(", Position: (");
        print_util_dbg_putfloat(new_goal.local_pos()[X], 3);
        print_util_dbg_print(", ");
        print_util_dbg_putfloat(new_goal.local_pos()[Y], 3);
        print_util_dbg_print(", ");
        print_util_dbg_putfloat(new_goal.local_pos()[Z], 3);
        print_util_dbg_print(")\r\n");

        if (navigation_strategy == strategy_t::DUBIN)
        {
            print_util_dbg_print("Reinitializing dubin\r\n");
            dubin_state = DUBIN_INIT;
        }
    }

    // Update goal based on the inputted waypoint
    goal_ = new_goal;
}

void Navigation::set_start_wpt_time()
{
    start_wpt_time_ = time_keeper_get_ms();
}

uint32_t Navigation::start_wpt_time() const
{
    return start_wpt_time_;
}

bool Navigation::waiting_at_waypoint() const
{
    return waiting_at_waypoint_;
}

void Navigation::set_waiting_at_waypoint(bool waiting_at_waypoint)
{
    waiting_at_waypoint_ = waiting_at_waypoint;
}
