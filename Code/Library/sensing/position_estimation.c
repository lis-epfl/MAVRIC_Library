/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file position_estimation.c
 *
 * This file performs the 3D position estimation, either by direct integration or with correction with the GPS and pos_est->barometer
 */


#include "position_estimation.h"
#include "print_util.h"
#include "math_util.h"
#include "time_keeper.h"
#include "conf_constants.h"
#include "conf_platform.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Direct integration of the position with the IMU data
 *
 * \param	pos_est					The pointer to the position estimation structure
 */
static void position_estimation_position_integration(position_estimator_t *pos_est);


/**
 * \brief	Position correction with the GPS and the barometer
 *
 * \param	pos_est					The pointer to the position estimation structure
 */
static void position_estimation_position_correction(position_estimator_t *pos_est);


/**
 * \brief	Initialization of the position estimation from the GPS position
 *
 * \param	pos_est			The pointer to the position estimation structure
 * \param	gps				The pointer to the GPS structure
 *
 * \return	void
 */
static void gps_position_init(position_estimator_t *pos_est);


/**
 * \brief	Position estimation update step, performing position estimation then position correction (function to be used)
 *
 * \param	pos_est					The pointer to the position estimation structure
 * \param	packet					The pointer to the decoded mavlink command long message
 */
static void position_estimation_set_new_home_position(position_estimator_t *pos_est, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void position_estimation_position_integration(position_estimator_t *pos_est)
{
	int32_t i;
	float dt = pos_est->ahrs->dt;
	
	quat_t qvel_bf,qvel; 

	qvel.s = 0;
	for (i = 0; i < 3; i++)
	{
		qvel.v[i] = pos_est->vel[i];
	}
	qvel_bf = quaternions_global_to_local(pos_est->ahrs->qe, qvel);
	for (i = 0; i < 3; i++)
	{
		pos_est->vel_bf[i] = qvel_bf.v[i];
		pos_est->vel_bf[i] = pos_est->vel_bf[i] * (1.0f - (VEL_DECAY * dt)) + pos_est->ahrs->linear_acc[i]  * dt;
	}
	
	// calculate velocity in global frame
	// vel = qe *vel_bf * qe - 1
	
	qvel_bf.s = 0.0f; 
	qvel_bf.v[0] = pos_est->vel_bf[0]; 	// TODO: replace this by quat_rotate_vector()
	qvel_bf.v[1] = pos_est->vel_bf[1]; 
	qvel_bf.v[2] = pos_est->vel_bf[2];
	
	qvel = quaternions_local_to_global(pos_est->ahrs->qe, qvel_bf);
	
	pos_est->vel[0] = qvel.v[0]; 
	pos_est->vel[1] = qvel.v[1]; 
	pos_est->vel[2] = qvel.v[2];	// TODO: replace this by quat_rotate_vector()
	
	for (i = 0; i < 3; i++)
	{
		pos_est->local_position.pos[i] = pos_est->local_position.pos[i] * (1.0f - (POS_DECAY * dt)) + pos_est->vel[i] * dt;

		pos_est->local_position.heading = coord_conventions_get_yaw(pos_est->ahrs->qe);
	}
	
}


static void position_estimation_position_correction(position_estimator_t *pos_est)
{
	global_position_t global_gps_position;
	local_coordinates_t local_coordinates;
	
	float dt = pos_est->ahrs->dt;
	
	// quat_t bias_correction = {.s = 0, .v = {0.0f, 0.0f, 1.0f}};
	quat_t vel_correction = 
	{
		.s = 0, 
		.v = 
		{
			0.0f, 
			0.0f, 
			1.0f
		}
	};

	float pos_error[3] = 
	{
		0.0f,
		0.0f,
		0.0f
	};
	
	float baro_alt_error = 0.0f;
	float baro_vel_error = 0.0f;
	float baro_gain = 0.0f;
	float gps_gain = 0.0f;
	float gps_dt = 0.0f;
	
	float vel_error[3] = 
	{
		0.0f,
		0.0f,
		0.0f
	};

	uint32_t t_inter_gps, t_inter_baro;
	int32_t i;

	if (pos_est->init_barometer)
	{		
		// altimeter correction
		if ( pos_est->time_last_barometer_msg < pos_est->barometer->last_update )
		{
			pos_est->last_alt = -(pos_est->barometer->altitude ) + pos_est->local_position.origin.altitude;
			baro_alt_error = -(pos_est->barometer->altitude ) - pos_est->local_position.pos[2] + pos_est->local_position.origin.altitude;

			pos_est->time_last_barometer_msg = pos_est->barometer->last_update;
		}

		t_inter_baro = (time_keeper_get_micros() - pos_est->barometer->last_update) / 1000.0f;
		baro_gain = 1.0f; //math_util_fmax(1.0f - t_inter_baro / 1000.0f, 0.0f);
			
		//pos_est->local_position.pos[2] += kp_alt_baro / ((float)(t_inter_baro / 2.5f + 1.0f)) * alt_error;
		baro_alt_error = pos_est->last_alt  - pos_est->local_position.pos[2];
		baro_vel_error = pos_est->barometer->vario_vz - pos_est->vel[2];
		//vel_error[2] = 0.1f * pos_error[2];
		//pos_est->vel[2] += kp_alt_baro_v * vel_error[2];
				
	}
	else
	{
		bmp085_reset_origin_altitude(pos_est->barometer, pos_est->local_position.origin.altitude);
		pos_est->init_barometer = true;
	}
	
	if (pos_est->init_gps_position)
	{
		if ( (pos_est->time_last_gps_msg < pos_est->gps->time_last_msg) && (pos_est->gps->status == GPS_OK) )
		{
			pos_est->time_last_gps_msg = pos_est->gps->time_last_msg;

			global_gps_position.longitude = pos_est->gps->longitude;
			global_gps_position.latitude = pos_est->gps->latitude;
			global_gps_position.altitude = pos_est->gps->altitude;
			global_gps_position.heading = 0.0f;
			local_coordinates = coord_conventions_global_to_local_position(global_gps_position,pos_est->local_position.origin);
			local_coordinates.timestamp_ms = pos_est->gps->time_last_msg;
			
			// compute GPS velocity estimate
			gps_dt = (local_coordinates.timestamp_ms - pos_est->last_gps_pos.timestamp_ms) / 1000.0f;
			if (gps_dt > 0.001f)
			{
				for (i = 0; i < 3; i++)
				{
					pos_est->last_vel[i] = (local_coordinates.pos[i] - pos_est->last_gps_pos.pos[i]) / gps_dt;
				}
				pos_est->last_gps_pos = local_coordinates;
			}
			else
			{
				print_util_dbg_print("GPS dt is too small!");
			}
		}
		t_inter_gps = time_keeper_get_millis() - pos_est->gps->time_last_msg;
			
		//gps_gain = math_util_fmax(1.0f - t_inter_gps / 1000.0f, 0.0f);
		gps_gain = 1.0f;
			
		for (i = 0;i < 3;i++)
		{
			pos_error[i] = pos_est->last_gps_pos.pos[i] - pos_est->local_position.pos[i];
			vel_error[i] = pos_est->last_vel[i]       - pos_est->vel[i]; 
		}
	}
	else
	{
		gps_position_init(pos_est);
		for (i = 0;i < 2;i++)
		{
			pos_error[i] = 0.0f;
			vel_error[i] = 0.0f;
		}
		gps_gain = 0.1f;
	}
		
	// Apply error correction to velocity and position estimates
	for (i = 0;i < 3;i++)
	{
		pos_est->local_position.pos[i] += pos_est->kp_pos_gps[i] * gps_gain * pos_error[i]* dt;
	}
	pos_est->local_position.pos[2] += pos_est->kp_alt_baro * baro_gain * baro_alt_error* dt;


	for (i = 0; i < 3; i++)
	{
		vel_correction.v[i] = vel_error[i];
	}
				
	for (i = 0;i < 3;i++)
	{			
		pos_est->vel[i] += pos_est->kp_vel_gps[i] * gps_gain * vel_correction.v[i]* dt;
	}
	pos_est->vel[2] += pos_est->kp_vel_baro * baro_gain * baro_vel_error* dt;
}


static void gps_position_init(position_estimator_t *pos_est)
{
	int32_t i;
	if ( pos_est->init_gps_position == false )
	{
		if ( pos_est->time_last_gps_msg < pos_est->gps->time_last_msg )
		{
			pos_est->time_last_gps_msg = pos_est->gps->time_last_msg;
		
			pos_est->init_gps_position = true;
			
			pos_est->local_position.origin.longitude = pos_est->gps->longitude;
			pos_est->local_position.origin.latitude = pos_est->gps->latitude;
			pos_est->local_position.origin.altitude = pos_est->gps->altitude;
			pos_est->local_position.timestamp_ms = pos_est->gps->time_last_msg;

			pos_est->last_gps_pos = pos_est->local_position;
			
			pos_est->last_alt = 0;
			for(i = 0;i < 3;i++)
			{
				pos_est->last_vel[i] = 0.0f;
				pos_est->local_position.pos[i] = 0.0f;
				pos_est->vel[i] = 0.0f;
			}
			
			print_util_dbg_print("GPS position initialized!\n");
		}
	}
}


static void position_estimation_set_new_home_position(position_estimator_t *pos_est, mavlink_command_long_t* packet)
{
	if (packet->param1 == 1)
 	{
 		// Set new home position to actual position
 		print_util_dbg_print("Set new home location to actual position.\n");
 		pos_est->local_position.origin = coord_conventions_local_to_global_position(pos_est->local_position);

 		print_util_dbg_print("New Home location: (");
 		print_util_dbg_print_num(pos_est->local_position.origin.latitude * 10000000.0f,10);
 		print_util_dbg_print(", ");
 		print_util_dbg_print_num(pos_est->local_position.origin.longitude * 10000000.0f,10);
 		print_util_dbg_print(", ");
 		print_util_dbg_print_num(pos_est->local_position.origin.altitude * 1000.0f,10);
 		print_util_dbg_print(")\n");
 	}
 	else
 	{
 		// Set new home position from msg
 		print_util_dbg_print("Set new home location. \n");

 		pos_est->local_position.origin.latitude = packet->param5;
 		pos_est->local_position.origin.longitude = packet->param6;
 		pos_est->local_position.origin.altitude = packet->param7;

 		print_util_dbg_print("New Home location: (");
 		print_util_dbg_print_num(pos_est->local_position.origin.latitude * 10000000.0f,10);
 		print_util_dbg_print(", ");
 		print_util_dbg_print_num(pos_est->local_position.origin.longitude * 10000000.0f,10);
 		print_util_dbg_print(", ");
 		print_util_dbg_print_num(pos_est->local_position.origin.altitude * 1000.0f,10);
 		print_util_dbg_print(")\n");
 	}

	*pos_est->waypoint_set = false;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void position_estimation_init(position_estimator_t *pos_est, barometer_t *barometer, gps_t *gps, ahrs_t *ahrs, imu_t *imu, const mavlink_stream_t* mavlink_stream, bool* waypoint_set, mavlink_message_handler_t *message_handler, float home_lat, float home_lon, float home_alt, float gravity)
{
    int32_t i;

	pos_est->barometer = barometer;
	pos_est->gps = gps;
	pos_est->ahrs = ahrs;
	pos_est->imu = imu;
	pos_est->mavlink_stream = mavlink_stream;
	pos_est->waypoint_set = waypoint_set;
	
	// default GPS home position
	pos_est->local_position.origin.longitude =   home_lon;
	pos_est->local_position.origin.latitude =   home_lat;
	pos_est->local_position.origin.altitude =   home_alt;
	pos_est->local_position.pos[X] = 0;
	pos_est->local_position.pos[Y] = 0;
	pos_est->local_position.pos[Z] = 0;
	
    // reset position estimator
    pos_est->last_alt = 0;
    for(i = 0;i < 3;i++)
    {
        pos_est->last_vel[i] = 0.0f;
        pos_est->vel[i] = 0.0f;
        pos_est->vel_bf[i] = 0.0f;
    }

	pos_est->gravity = gravity;
	
	pos_est->init_gps_position = false;
	pos_est->init_barometer = false;
	pos_est->time_last_gps_msg = 0;
	pos_est->time_last_barometer_msg = 0;
	
    pos_est->kp_pos_gps[X] = 2.0f;
    pos_est->kp_pos_gps[Y] = 2.0f;
    pos_est->kp_pos_gps[Z] = 1.0f;

    pos_est->kp_vel_gps[X] = 1.0f;
    pos_est->kp_vel_gps[Y] = 1.0f;
    pos_est->kp_vel_gps[Z] = 0.5f;
	
	pos_est->kp_alt_baro = 2.0f;
	pos_est->kp_vel_baro = 1.0f;
	
	gps_position_init(pos_est);
	
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	callbackcmd.command_id    = MAV_CMD_DO_SET_HOME; // 179
	callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER;
	callbackcmd.function      = (mavlink_cmd_callback_function_t)	&position_estimation_set_new_home_position;
	callbackcmd.module_struct =										pos_est;
	mavlink_message_handler_add_cmd_callback(message_handler, &callbackcmd);
	
	print_util_dbg_print("Position estimation initialized.\n");
}


void position_estimation_reset_home_altitude(position_estimator_t *pos_est)
{
	int32_t i;
	// reset origin to position where quad is armed if we have GPS
	if (pos_est->init_gps_position)
	{
		pos_est->local_position.origin.longitude = pos_est->gps->longitude;
		pos_est->local_position.origin.latitude = pos_est->gps->latitude;
		pos_est->local_position.origin.altitude = pos_est->gps->altitude;
		pos_est->local_position.timestamp_ms = pos_est->gps->time_last_msg;

		pos_est->last_gps_pos = pos_est->local_position;
	}
	//else
	//{
		//pos_est->local_position.origin.longitude = HOME_LONGITUDE;
		//pos_est->local_position.origin.latitude = HOME_LATITUDE;
		//pos_est->local_position.origin.altitude = HOME_ALTITUDE;
	//}

	// reset barometer offset
	bmp085_reset_origin_altitude(pos_est->barometer, pos_est->local_position.origin.altitude);

	//pos_est->barometer->altitude_offset = -pos_est->barometer->altitude - pos_est->local_position.pos[2] + pos_est->local_position.origin.altitude;
	pos_est->init_barometer = true;
		
	print_util_dbg_print("Offset of the barometer set to the GPS altitude, offset value of:");
	print_util_dbg_print_num(pos_est->barometer->altitude_offset,10);
	print_util_dbg_print(" = -");
	print_util_dbg_print_num(pos_est->barometer->altitude,10);
	print_util_dbg_print(" - ");
	print_util_dbg_print_num(pos_est->local_position.pos[2],10);
	print_util_dbg_print(" + ");
	print_util_dbg_print_num(pos_est->local_position.origin.altitude,10);
	print_util_dbg_print("\n");

	// reset position estimator
	pos_est->last_alt = 0;
	for(i = 0;i < 3;i++)
	{
		pos_est->last_vel[i] = 0.0f;
		pos_est->local_position.pos[i] = 0.0f;
		pos_est->vel[i] = 0.0f;
		pos_est->vel_bf[i] = 0.0f;
	}
}


void position_estimation_update(position_estimator_t *pos_est)
{
	//if (attitude_filter->calibration_level == OFF)
	{
		position_estimation_position_integration(pos_est);
		position_estimation_position_correction(pos_est);
	}
}


task_return_t position_estimation_send_position(position_estimator_t* pos_est)
{
	mavlink_message_t msg;
	mavlink_msg_local_position_ned_pack(	pos_est->mavlink_stream->sysid,
											pos_est->mavlink_stream->compid,
											&msg,
											time_keeper_get_millis(),
											pos_est->local_position.pos[0],
											pos_est->local_position.pos[1],
											pos_est->local_position.pos[2],
											pos_est->vel[0],
											pos_est->vel[1],
											pos_est->vel[2]);
	mavlink_stream_send(pos_est->mavlink_stream, &msg);

	return TASK_RUN_SUCCESS;
}

task_return_t position_estimation_send_global_position(position_estimator_t* pos_est)
{
	// send integrated position (for now there is no GPS error correction...!!!)
	global_position_t gpos = coord_conventions_local_to_global_position(pos_est->local_position);
	
	//mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
	mavlink_message_t msg;
	mavlink_msg_global_position_int_pack(	pos_est->mavlink_stream->sysid,
											pos_est->mavlink_stream->compid,
											&msg,
											time_keeper_get_millis(),
											gpos.latitude * 10000000,
											gpos.longitude * 10000000,
											gpos.altitude * 1000.0f,
											-pos_est->local_position.pos[2] * 1000,
											pos_est->vel[0] * 100.0f,
											pos_est->vel[1] * 100.0f,
											pos_est->vel[2] * 100.0f,
											pos_est->local_position.heading);
	mavlink_stream_send(pos_est->mavlink_stream, &msg);
	
	return TASK_RUN_SUCCESS;
}