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
 * \file lequad_symbiotic.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/

#include "sample_projects/LEQuad/lequad_symbiotic.hpp"
#include "sample_projects/LEQuad/lequad.hpp"


LEQuad_symbiotic::LEQuad_symbiotic(
	Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink,
    Satellite& satellite, Led& led, File& file_flash, Battery& battery,
    Servo& servo_0, Servo& servo_1, Servo& servo_2, Servo& servo_3, Servo& servo_4,
    Servo& servo_5, Servo& servo_6, Servo& servo_7, File& file1, File& file2, const conf_t& config):

	Base_class(
	imu, barometer, gps, sonar, serial_mavlink, satellite, led, file_flash,
    battery, servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7,
    file1, file2, config.lequad_config
	),
	gimbal_controller(navigation, servo_5, servo_4, config.gimbal_controller_config)
{
	init_gimbal();
};

// -------------------------------------------------------------------------
// Main task
// -------------------------------------------------------------------------
bool LEQuad_symbiotic::main_task(void)
{
	//--Alex
	//gimbal_controller.get_fake_pitch(&stabilisation_copter);
	//--

    // Update estimation
    imu.update();
    ahrs_ekf.update();
    position_estimation.update();

    bool failsafe = false;

    //-- Alex -- If the remote is in manual mode, put the Quad in manual mode
    //Get current mode
	Mav_mode current_mode = state.mav_mode();
	//Get mode from the remote
	Mav_mode mode_remote = manual_control.manual_control_get_mode_from_remote(current_mode);
	//print_util_dbg_putfloat((int) mode_remote.bits(),3);
	//print_util_dbg_print("\r\n");

	//if the mode is manual set the current mode to manual
	if(manual_control.control_source() == Manual_control::CONTROL_SOURCE_JOYSTICK && remote_check(&manual_control.remote) != SIGNAL_LOST && (((int) mode_remote.bits() == 80) || ((int) mode_remote.bits() == 208)))
	{
		state.mav_mode_.set_ctrl_mode(Mav_mode::ATTITUDE);
		manual_control.set_mode_source(Manual_control::MODE_SOURCE_REMOTE);
	}
	else if(manual_control.control_source() == Manual_control::CONTROL_SOURCE_JOYSTICK)
		manual_control.set_mode_source(Manual_control::MODE_SOURCE_GND_STATION);
    //---

    // Do control
    if (state.is_armed())
    {
        switch (state.mav_mode().ctrl_mode())
        {
            case Mav_mode::GPS_NAV:
                controls = controls_nav;
                controls.control_mode = VELOCITY_COMMAND_MODE;

                //if still on ground, don't move and wait on takeoff
				if(navigation.internal_state_ == Navigation::NAV_ON_GND)
				{
					controls.rpy[ROLL] = 0.0f;
					controls.rpy[PITCH] = 0.0f;
					controls.rpy[YAW] = 0.0f;
					controls.thrust = -0.7f;
					controls.yaw_mode = YAW_RELATIVE;
					controls.control_mode = ATTITUDE_COMMAND_MODE;
					/*controls.rpy[ROLL] = 0.0f;
					controls.rpy[PITCH] = 0.0f;
					controls.rpy[YAW] = 0.0f;
					controls.thrust = -0.7f;
					controls.yaw_mode = YAW_RELATIVE;
					controls.control_mode = RATE_COMMAND_MODE;*/
				}

                // if no waypoints are set, we do position hold therefore the yaw mode is absolute
                if ((((state.nav_plan_active || (navigation.navigation_strategy == Navigation::strategy_t::DUBIN)) && (navigation.internal_state_ == Navigation::NAV_NAVIGATING)) || (navigation.internal_state_ == Navigation::NAV_STOP_THERE))
              	   || ((state.mav_state_ == MAV_STATE_CRITICAL) && (navigation.critical_behavior == Navigation::FLY_TO_HOME_WP)))
            	{
                    controls.yaw_mode = YAW_RELATIVE;
                }
                else
                {
                    controls.yaw_mode = YAW_ABSOLUTE;
                }
                break;

            case Mav_mode::POSITION_HOLD:
            	print_util_dbg_print("position_hold");
                controls = controls_nav;
                controls.control_mode = VELOCITY_COMMAND_MODE;

                if ( ((state.mav_state_ == MAV_STATE_CRITICAL) && (navigation.critical_behavior == Navigation::FLY_TO_HOME_WP))  || (navigation.navigation_strategy == Navigation::strategy_t::DUBIN))
                {
                    controls.yaw_mode = YAW_RELATIVE;
                }
                else
                {
                    controls.yaw_mode = YAW_ABSOLUTE;
                }
                break;

            case Mav_mode::VELOCITY:
                manual_control.get_velocity_vector(&controls);
                controls.control_mode = VELOCITY_COMMAND_MODE;
                controls.yaw_mode = YAW_RELATIVE;
                break;

            case Mav_mode::ATTITUDE:
            {
            	//Alex----
				//Control source and mode source is forced to be taken from the remote
				Manual_control::control_source_t chosen_control_source = manual_control.control_source();
				manual_control.set_control_source(Manual_control::CONTROL_SOURCE_REMOTE);
					//-----

				manual_control.get_control_command(&controls);
                controls.control_mode = ATTITUDE_COMMAND_MODE;
                controls.yaw_mode = YAW_RELATIVE;

                //Alex----
                manual_control.set_control_source(chosen_control_source);
                //----
                break;

            //case Mav_mode::RATE:
            //    manual_control.get_rate_command(&controls);
            //    controls.control_mode = RATE_COMMAND_MODE;
            //    break;
            }
            case Mav_mode::COSTUM:
			{
            	manual_control.manual_control_get_from_joystick_symbiotic(&controls);

				float max_vel_z = 5.5f; //[m/s]
				float max_vel_y = 5.5f; //[m/s]

				// fence avoidance
				bool compute_repulsion = false;
				float xy_dist;
				float z_dist;
				float norm_ctrl_vel_xy_sqr;
				float origin2quad[3];
				float quadOrientation[3];
				float out[3];

				bool in_drone_dome_fence = false;

				bool outside_fence;


				global_position_t fence_origin_global;
				local_position_t fence_origin_local;

				fence_origin_global.altitude = 400.0f;
				fence_origin_global.latitude = 46.5187539;
				fence_origin_global.longitude = 6.5667397f;
				fence_origin_global.heading = 0.0f;
				fence_origin_local = coord_conventions_global_to_local_position(fence_origin_global,position_estimation.local_position.origin);

				if(_f_outside > 0.5f)
					outside_fence = true;
				else
					outside_fence = false;

				if(in_drone_dome_fence) //ideal speed 0.5 to 0.7 m/s
				{
					//center point (0,0,-2.0)
					z_min = -0.75f;//[m]
					z_max = -3.5f;//[m]
					xy_max = 2.75f; //radius [m]
					dist_to_limit_lat = 0.75f; //[m]
					dist_to_limit_ver = 0.75f; //[m]
					compute_repulsion = true;
				}
				else if(outside_fence)
				{
					//center point (0,0,-10.0)
					//z_min = -5.0f;//[m]
					//z_max = -20.0f;//[m]
					//xy_max = 20.0f; //radius [m]
					//dist_to_limit = 5.0f; //[m]
					compute_repulsion = true;
				}

				if(compute_repulsion)
				{
					// ---------------------------- Vertical Fence -------------------------------

					origin2quad[0] = 0.0f;
					origin2quad[1] = 0.0f;
					origin2quad[2] = position_estimation.local_position.pos[Z] - (z_min + z_max)/2;
					quadOrientation[0] = 0.0f;
					quadOrientation[1] = 0.0f;
					quadOrientation[2] = position_estimation.vel[2];

					float decrease_factor_Z = 1.0f;
					if(SCP(origin2quad,quadOrientation) < 0.0f) //the drone as a velocity toward the center => decrease the avoidance speed
						decrease_factor_Z = 1.0f;

					//print_util_dbg_putfloat(decrease_factor_Z,3);
					//print_util_dbg_print("\r\n");


					xy_dist = maths_fast_sqrt(SQR(position_estimation.local_position.pos[X] - fence_origin_local.pos[X]) +
								SQR(position_estimation.local_position.pos[Y] - fence_origin_local.pos[Y]));
					z_dist = position_estimation.local_position.pos[Z];

					//xy_dist = position_estimation.position_estimation_get_xy_distance_from_fence_origin();
					//z_dist = position_estimation.position_estimation_get_z_distance_from_fence_origin();

					norm_ctrl_vel_xy_sqr = SQR(controls.tvel[X]) + SQR(controls.tvel[Y]);

					//Vertical fence
					//float ratioXZ_vel = 1.0f;
					if(z_dist < (z_max + dist_to_limit_ver)) //if too high => tvel_z_added is positif
					{
						float ratio = maths_f_abs(z_dist - (z_max+dist_to_limit_ver)) / dist_to_limit_ver;
						if(ratio > 1.0f)
							ratio = 1.0f;

						float tvel_z_added = ratio * max_vel_z;

						//decrease if going back to center
						tvel_z_added = tvel_z_added*decrease_factor_Z;

						//if(maths_f_abs(tvel_z_added + central_data->controls.tvel[Z])/maths_fast_sqrt(norm_ctrl_vel_xy_sqr) > ratioXZ_vel)
						//					tvel_z_added = sign(tvel_z_added)*maths_fast_sqrt(norm_ctrl_vel_xy_sqr)*ratioXZ_vel - central_data->controls.tvel[Z];

						controls.tvel[Z] += tvel_z_added;
						//if(z_dist < z_max)
						//		central_data->controls.tvel[Z] = max_vel_z;
					}
					if(z_dist > (z_min - dist_to_limit_ver)) //if too low => tvel_z_added is negatif
					{
						float ratio = -maths_f_abs(z_dist - (z_min-dist_to_limit_ver)) / dist_to_limit_ver;
						if(ratio > 1.0f)
							ratio = 1.0f;

						float tvel_z_added = ratio * max_vel_z;

						//decrease if going back to center
						tvel_z_added = tvel_z_added*decrease_factor_Z;

						//if(maths_f_abs(tvel_z_added + central_data->controls.tvel[Z])/maths_fast_sqrt(norm_ctrl_vel_xy_sqr) > ratioXZ_vel)
						//					tvel_z_added = sign(tvel_z_added)*maths_fast_sqrt(norm_ctrl_vel_xy_sqr)*ratioXZ_vel - central_data->controls.tvel[Z];

						controls.tvel[Z] += tvel_z_added;

						//if(z_dist > z_min)
						//		central_data->controls.tvel[Z] = -max_vel_z;
					}


					// ------------------------- Lateral Fence --------------------------

					origin2quad[0] = position_estimation.local_position.pos[X] - fence_origin_local.pos[X];
					origin2quad[1] = position_estimation.local_position.pos[Y] - fence_origin_local.pos[Y];
					//origin2quad[0] = position_estimation.local_position.pos[X] - position_estimation.get_fence_position().pos[X];
					//origin2quad[1] = position_estimation.local_position.pos[Y] - position_estimation.get_fence_position().pos[Y];
					origin2quad[2] = 0.0f;

					quadOrientation[0] = position_estimation.vel[0];
					quadOrientation[1] = position_estimation.vel[1];
					quadOrientation[2] = 0.0f;

					CROSS(origin2quad,quadOrientation,out);

					float decrease_factor_Y = 1.0f;
					if(SCP(origin2quad,quadOrientation) < 0.0f) //the drone as a velocity toward the center => decrease the avoidance speed
						decrease_factor_Y = 0.4f;

					//print_util_dbg_print("Y then Z\r\n");
					//print_util_dbg_putfloat(decrease_factor_Y,3);
					//print_util_dbg_print("\r\n");

					float tvel_y_added;
					float ratioXY_vel = 0.6f; //should be between 0.0f and 1.0f
					if(xy_dist > (xy_max - dist_to_limit_lat))
					{
						//compute repulsion velocity y
						float ratio = maths_f_abs(xy_dist - (xy_max-dist_to_limit_lat)) / dist_to_limit_lat;
						if(ratio > 1.0f)
							ratio = 1.0f;

						//decrease ratio if coming back to the center
						if(xy_dist > xy_max)
						{
							float origin2quad_normalized[3];
							float quadOrientation_normalized[3];
							vectors_normalize(origin2quad,origin2quad_normalized);
							vectors_normalize(quadOrientation,quadOrientation_normalized);
							float scp = SCP(origin2quad_normalized,quadOrientation_normalized); //-1 if opposite direction, 1 same direction

							if(scp < 0.0f) //going toward the center
								ratio = ratio * (1 + scp);
						}
						else
							ratio = ratio * decrease_factor_Y;

						print_util_dbg_print("ratio \r\n");
						print_util_dbg_putfloat(ratio,3);
						print_util_dbg_print("\r\n");

						tvel_y_added = sign(out[2])* ratio * max_vel_y;

						//troncate total velocity y to a ratio of the norm of the total desired speed
						if(maths_f_abs(tvel_y_added + controls.tvel[Y])/maths_fast_sqrt(norm_ctrl_vel_xy_sqr) > ratioXY_vel)
							tvel_y_added = sign(tvel_y_added)*maths_fast_sqrt(norm_ctrl_vel_xy_sqr)*ratioXY_vel - controls.tvel[Y];

						controls.tvel[Y] += tvel_y_added;

						//do this to ensure that whatever input in tvel[Y] we have the tvel[Y] output not bigger than the norm
						if(controls.tvel[Y] > 0.0f && SQR(controls.tvel[Y]) > norm_ctrl_vel_xy_sqr + 0.001f)
							controls.tvel[Y] = maths_fast_sqrt(norm_ctrl_vel_xy_sqr);
						else if(controls.tvel[Y] < 0.0f && SQR(controls.tvel[Y]) > norm_ctrl_vel_xy_sqr + 0.001f)
							controls.tvel[Y] = -maths_fast_sqrt(norm_ctrl_vel_xy_sqr);

						print_util_dbg_print("tvel_y_added \r\n");
						print_util_dbg_putfloat(tvel_y_added,3);
						print_util_dbg_print("\r\n");

						//reduce the speed on tvel[X] in order to keep the norm of the speed constant
						controls.tvel[X] = maths_fast_sqrt(norm_ctrl_vel_xy_sqr - SQR(controls.tvel[Y]));
					}
				}

				/*print_util_dbg_print("vel cmd\r\n");
				print_util_dbg_print("Vx ");
				print_util_dbg_putfloat(controls.tvel[X],3);
				print_util_dbg_print("  Vy ");
				print_util_dbg_putfloat(controls.tvel[Y],3);
				print_util_dbg_print("  Vz ");
				print_util_dbg_putfloat(controls.tvel[Z],3);
				print_util_dbg_print("  Yaw ");
				print_util_dbg_putfloat(controls.rpy[YAW],3);
				print_util_dbg_print("\r\n");*/

				controls.control_mode = VELOCITY_COMMAND_MODE;

				//YAW_COORDINATED to have the yaw in the axis of the velocity speed
				controls.yaw_mode = YAW_COORDINATED;

				break;
			}
            default:
                failsafe = true;    // undefined behaviour -> failsafe

        }
    }
    else    // !state.is_armed()
    {
        failsafe = true;    // undefined behaviour -> failsafe
    }

    // if behaviour defined, execute controller and mix; otherwise: set servos to failsafe
    if(!failsafe)
    {
        stabilisation_copter_cascade_stabilise(&stabilisation_copter);
        servos_mix_quadcopter_diag_update(&servo_mix);
    }
    else
    {
        servo_0.failsafe();
        servo_1.failsafe();
        servo_2.failsafe();
        servo_3.failsafe();
    }

    return true;
}

// -------------------------------------------------------------------------
// GIMBAL
// -------------------------------------------------------------------------
bool LEQuad_symbiotic::init_gimbal(void)
{

    z_min = -5.0f;//[m]
	z_max = -20.0f;//[m]
	xy_max = 20.0f; //radius [m]
	dist_to_limit_lat = 5.0f; //[m]
	dist_to_limit_ver = 5.0f; //[m]
	_f_outside = 0.0f;

    bool ret = true;

    // Parameters
    Onboard_parameters& op = mavlink_communication.onboard_parameters();
    ret &= op.add_parameter_float(&stabilisation_copter.w_,"_w_");
    ret &= op.add_parameter_float(&z_min,"_f_z_min");
    ret &= op.add_parameter_float(&z_max,"_f_z_max");
    ret &= op.add_parameter_float(&xy_max,"_f_xy_max");
    ret &= op.add_parameter_float(&dist_to_limit_lat,"_f_dtl_lat");
    ret &= op.add_parameter_float(&dist_to_limit_ver,"_f_dtl_ver");
    ret &= op.add_parameter_float(&_f_outside,"_f_outside");

    // UP telemetry
    ret &= gimbal_controller_telemetry_init(&gimbal_controller, mavlink_communication.p_message_handler());

    // DOWN telemetry

    // Task
    ret &= scheduler.add_task(10000, (Scheduler_task::task_function_t)&Gimbal_controller::update, (Scheduler_task::task_argument_t)&gimbal_controller, Scheduler_task::PRIORITY_NORMAL);

    // Data logging
    ret &= data_logging_stat.add_field(&controls.tvel[X], "tvel_x", 3);
    ret &= data_logging_stat.add_field(&controls.tvel[Y], "tvel_y", 3);
    ret &= data_logging_stat.add_field(&controls.tvel[Z], "tvel_z", 3);
    //ret &= data_logging_stat.add_field(&controls.rpy[0], "in_roll", 3);
    //ret &= data_logging_stat.add_field(&controls.rpy[1], "in_pitch", 3);
    //ret &= data_logging_stat.add_field(&gimbal_controller.roll_body_, "out_roll", 3);
    //ret &= data_logging_stat.add_field(&gimbal_controller.fake_pitch_, "out_f_pitch", 3);
    //ret &= data_logging_stat.add_field(&gimbal_controller.vel_x_semilocal_, "vel_x_sl", 3);
    //ret &= data_logging_stat.add_field(&manual_control.joystick.commTrigger, "p_joy_ms", 3);
    //ret &= data_logging_stat.add_field(&gimbal_controller.commTrig_, "p_gim_ms", 3);

    return ret;
}
