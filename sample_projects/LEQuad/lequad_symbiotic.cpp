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
	gimbal_controller(navigation, servo_5, servo_6, config.gimbal_controller_config)
{
	init_gimbal();
};

// -------------------------------------------------------------------------
// Main task
// -------------------------------------------------------------------------
bool LEQuad_symbiotic::main_task(void)
{
	//--Alex
	gimbal_controller.get_fake_pitch(&stabilisation_copter);
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
	if(remote_check(&manual_control.remote) != SIGNAL_LOST && (((int) mode_remote.bits() == 80) || ((int) mode_remote.bits() == 208)))
	{
		state.mav_mode_.set_ctrl_mode(Mav_mode::ATTITUDE);
		manual_control.set_mode_source(Manual_control::MODE_SOURCE_REMOTE);
	}
	else
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

				float max_vel_z = 6.0f; //[m/s]
				float max_vel_y = 6.0f; //[m/s]

				// fence avoidance
				bool compute_repulsion = false;
				float xy_dist;
				float z_dist;
				float norm_ctrl_vel_xy_sqr;
				float z_min, z_max, xy_max, dist_to_limit;
				float origin2quad[3];
				float quadOrientation[3];
				float out[3];

				bool in_drone_dome = true;
				if(in_drone_dome) //ideal speed 0.5 to 0.7 m/s
				{
					//center point (0,0,2.0)
					z_min = -0.75f;//[m]
					z_max = -3.5f;//[m]
					xy_max = 3.0f; //radius [m]
					dist_to_limit = 0.5f; //[m]
					compute_repulsion = true;
				}
				else if(manual_control.joystick.isFenceEnabled)
				{
					z_min = 6.0f;//[m]  //underground
					z_max = -23.0f;//[m]
					xy_max = 18.0f; //radius [m]
					dist_to_limit = 3.0f; //[m]
					compute_repulsion = true;
				}

				if(compute_repulsion)
				{
					xy_dist = position_estimation.position_estimation_get_xy_distance_from_fence_origin();
					z_dist = position_estimation.position_estimation_get_z_distance_from_fence_origin();
					norm_ctrl_vel_xy_sqr = SQR(controls.tvel[X]) + SQR(controls.tvel[Y]);

					//Vertical fence
					//float ratioXZ_vel = 1.0f;
					if(z_dist < (z_max + dist_to_limit)) //if too high => tvel_z_added is positif
					{
						float tvel_z_added = SQR(z_dist - (z_max+dist_to_limit)) / dist_to_limit * max_vel_z;

						//if(maths_f_abs(tvel_z_added + central_data->controls.tvel[Z])/maths_fast_sqrt(norm_ctrl_vel_xy_sqr) > ratioXZ_vel)
						//					tvel_z_added = sign(tvel_z_added)*maths_fast_sqrt(norm_ctrl_vel_xy_sqr)*ratioXZ_vel - central_data->controls.tvel[Z];

						controls.tvel[Z] += tvel_z_added;
						//if(z_dist < z_max)
						//		central_data->controls.tvel[Z] = max_vel_z;
					}
					if(z_dist > (z_min - dist_to_limit)) //if too low => tvel_z_added is negatif
					{
						float tvel_z_added = -SQR(z_dist - (z_min-dist_to_limit)) / dist_to_limit * max_vel_z;

						//if(maths_f_abs(tvel_z_added + central_data->controls.tvel[Z])/maths_fast_sqrt(norm_ctrl_vel_xy_sqr) > ratioXZ_vel)
						//					tvel_z_added = sign(tvel_z_added)*maths_fast_sqrt(norm_ctrl_vel_xy_sqr)*ratioXZ_vel - central_data->controls.tvel[Z];

						controls.tvel[Z] += tvel_z_added;

						//if(z_dist > z_min)
						//		central_data->controls.tvel[Z] = -max_vel_z;
					}

					//Lateral fence
					origin2quad[0] = position_estimation.local_position.pos[X] - position_estimation.get_fence_position().pos[X];
					origin2quad[1] = position_estimation.local_position.pos[Y] - position_estimation.get_fence_position().pos[Y];
					origin2quad[2] = 0.0f;

					quadOrientation[0] = position_estimation.vel[0];
					quadOrientation[1] = position_estimation.vel[1];
					quadOrientation[2] = 0.0f;

					CROSS(origin2quad,quadOrientation,out);

					float tvel_y_added;
					float ratioXY_vel = 1.0f; //should be between 0.0f and 1.0f
					if(xy_dist > (xy_max - dist_to_limit))
					{
						//compute repulsion velocity y
						tvel_y_added = sign(out[2])*SQR(xy_dist - (xy_max-dist_to_limit)) / dist_to_limit * max_vel_y;

						//troncate repulsion velocity y to the norm of the total speed
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
    bool ret = true;

    // UP telemetry
    ret &= gimbal_controller_telemetry_init(&gimbal_controller, mavlink_communication.p_message_handler());

    // DOWN telemetry

    // Task
    ret &= scheduler.add_task(10000, (Scheduler_task::task_function_t)&Gimbal_controller::update, (Scheduler_task::task_argument_t)&gimbal_controller, Scheduler_task::PRIORITY_NORMAL);

    // Data logging
    //ret &= data_logging_continuous.add_field(&controls.rpy[0], "in_roll", 3);
    //ret &= data_logging_continuous.add_field(&controls.rpy[1], "in_pitch", 3);
    //ret &= data_logging_continuous.add_field(&gimbal_controller.roll_body_, "out_roll", 3);
    //ret &= data_logging_continuous.add_field(&gimbal_controller.fake_pitch_, "out_f_pitch", 3);
    //ret &= data_logging_continuous.add_field(&gimbal_controller.vel_x_semilocal_, "vel_x_sl", 3);
    //ret &= data_logging_continuous.add_field(&manual_control.joystick.commTrigger, "p_joy_ms", 3);
    //ret &= data_logging_continuous.add_field(&gimbal_controller.commTrig_, "p_gim_ms", 3);

    return ret;
}
