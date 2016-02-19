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
 * \file stabilisation_telemetry.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the stabilisation module
 *
 ******************************************************************************/


#include "control/stabilisation_telemetry.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/constants.h"
}

void  stabilisation_telemetry_send_rpy_speed_thrust_setpoint(const stabiliser_t* stabiliser, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack(mavlink_stream->sysid,
            mavlink_stream->compid,
            msg,
            time_keeper_get_ms(),
            stabiliser->rpy_controller[0].output,
            stabiliser->rpy_controller[1].output,
            stabiliser->rpy_controller[2].output,
            stabiliser->thrust_controller.output);
}

void  stabilisation_telemetry_send_rpy_rates_error(const stabiliser_t* stabiliser, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_pack(mavlink_stream->sysid,
            mavlink_stream->compid,
            msg,
            time_keeper_get_ms(),
            stabiliser->rpy_controller[0].error,
            stabiliser->rpy_controller[1].error,
            stabiliser->rpy_controller[2].error,
            stabiliser->thrust_controller.error);
}

void stabilisation_telemetry_send_rpy_thrust_setpoint(const control_command_t* controls, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    // Controls output
    mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack(mavlink_stream->sysid,
            mavlink_stream->compid,
            msg,
            time_keeper_get_ms(),
            controls->rpy[ROLL],
            controls->rpy[PITCH],
            controls->rpy[YAW],
            controls->thrust);
}

void stabilisation_telemetry_send_control(const control_command_t* controls, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    switch (controls->control_mode)
    {
        case VELOCITY_COMMAND_MODE:
            mavlink_msg_manual_control_pack(mavlink_stream->sysid,
                                            mavlink_stream->compid,
                                            msg,
                                            mavlink_stream->sysid,
                                            controls->tvel[X] * 1000,
                                            controls->tvel[Y] * 1000,
                                            controls->tvel[Z] * 1000,
                                            controls->rpy[YAW] * 1000,
                                            0);
            break;
        case ATTITUDE_COMMAND_MODE:
            mavlink_msg_manual_control_pack(mavlink_stream->sysid,
                                            mavlink_stream->compid,
                                            msg,
                                            mavlink_stream->sysid,
                                            controls->rpy[ROLL] * 1000,
                                            controls->rpy[PITCH] * 1000,
                                            controls->thrust * 1000,
                                            controls->rpy[YAW] * 1000,
                                            0);
            break;
        case RATE_COMMAND_MODE:
            mavlink_msg_manual_control_pack(mavlink_stream->sysid,
                                            mavlink_stream->compid,
                                            msg,
                                            mavlink_stream->sysid,
                                            controls->rpy[ROLL] * 1000,
                                            controls->rpy[PITCH] * 1000,
                                            controls->thrust * 1000,
                                            controls->rpy[YAW] * 1000,
                                            0);
    }
}

void stabilisation_copter_send_outputs(stabilisation_copter_t* stabilisation_copter, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    aero_attitude_t attitude_yaw_inverse;
    quat_t q_rot, qtmp;

    attitude_yaw_inverse = coord_conventions_quat_to_aero(stabilisation_copter->ahrs->qe);
    attitude_yaw_inverse.rpy[0] = 0.0f;
    attitude_yaw_inverse.rpy[1] = 0.0f;
    attitude_yaw_inverse.rpy[2] = attitude_yaw_inverse.rpy[2];

    q_rot = coord_conventions_quaternion_from_aero(attitude_yaw_inverse);
    qtmp = quaternions_create_from_vector(stabilisation_copter->stabiliser_stack.velocity_stabiliser.output.rpy);
    quat_t rpy_local;
    quaternions_rotate_vector(quaternions_inverse(q_rot), qtmp.v, rpy_local.v);

    mavlink_msg_debug_vect_pack(mavlink_stream->sysid,
                                mavlink_stream->compid,
                                msg,
                                "OutVel",
                                time_keeper_get_us(),
                                -rpy_local.v[X] * 1000,
                                rpy_local.v[Y] * 1000,
                                stabilisation_copter->stabiliser_stack.velocity_stabiliser.output.rpy[YAW] * 1000);
    mavlink_stream_send(mavlink_stream, msg);

    mavlink_msg_debug_vect_pack(mavlink_stream->sysid,
                                mavlink_stream->compid,
                                msg,
                                "OutAtt",
                                time_keeper_get_us(),
                                stabilisation_copter->stabiliser_stack.attitude_stabiliser.output.rpy[ROLL] * 1000,
                                stabilisation_copter->stabiliser_stack.attitude_stabiliser.output.rpy[PITCH] * 1000,
                                stabilisation_copter->stabiliser_stack.attitude_stabiliser.output.rpy[YAW] * 1000);
    mavlink_stream_send(mavlink_stream, msg);

    mavlink_msg_debug_vect_pack(mavlink_stream->sysid,
                                mavlink_stream->compid,
                                msg,
                                "OutRate",
                                time_keeper_get_us(),
                                stabilisation_copter->stabiliser_stack.rate_stabiliser.output.rpy[ROLL] * 1000,
                                stabilisation_copter->stabiliser_stack.rate_stabiliser.output.rpy[PITCH] * 1000,
                                stabilisation_copter->stabiliser_stack.rate_stabiliser.output.rpy[YAW] * 1000);
    mavlink_stream_send(mavlink_stream, msg);
}

void  sensors_set_telemetry_send(Central_data *central_data, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	float angle[3];
	aero_attitude_t aero_attitude;
	quat_t up = {0.0f, {UPVECTOR_X, UPVECTOR_Y, UPVECTOR_Z}};

	aero_attitude =  coord_conventions_quat_to_aero(quaternions_global_to_local(central_data->stabilisation_copter.ahrs->qe,up));
	angle[0] = aero_attitude.rpy[0];
	angle[1] = aero_attitude.rpy[1];
	angle[2] = aero_attitude.rpy[2];

	int16_t heading;

	if (angle[2] < 0)
	    {
	        heading = (int16_t)(360.0f + 180.0f * angle[2] / PI); //you want to normalize between 0 and 360°
	    }
	    else
	    {
	        heading = (int16_t)(180.0f * angle[2] / PI);
	    }

	mavlink_msg_angle_rate_velocity_sensors_pack(mavlink_stream->sysid,
            mavlink_stream->compid,
            msg,
            time_keeper_get_ms(),
			angle,
            central_data->ahrs.angular_speed,
			central_data->stabilisation_copter.pos_est->vel[X], //in global frame
			central_data->stabilisation_copter.pos_est->vel[Y], //in global frame
			central_data->stabilisation_copter.pos_est->vel[Z], //in global frame
			0.0f);
}
