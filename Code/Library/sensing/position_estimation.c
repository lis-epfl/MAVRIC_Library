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
 * \brief	Initialization of the pos_est->barometer offset
 *
 * \param	pos_est			The pointer to the position estimation structure
 * \param	pos_est->barometer		The pointer to the pos_est->barometer structure
 *
 * \return	void
 */
static void barometer_offset_init(position_estimator_t *pos_est);

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
	float dt = pos_est->attitude_estimation->dt;
	
	UQuat_t qvel_bf,qvel; 

	qvel.s = 0;
	for (i = 0; i < 3; i++)
	{
		qvel.v[i] = pos_est->vel[i];
	}
	qvel_bf = quaternions_global_to_local(pos_est->attitude_estimation->qe, qvel);
	for (i = 0; i < 3; i++)
	{
		pos_est->vel_bf[i] = qvel_bf.v[i];
		// clean acceleration estimate without gravity:
		pos_est->attitude_estimation->linear_acc[i] = pos_est->gravity * (pos_est->imu->scaled_accelero.data[i] - pos_est->attitude_estimation->up_vec.v[i]) ;							// TODO: review this line!
		pos_est->vel_bf[i] = pos_est->vel_bf[i] * (1.0f - (VEL_DECAY * dt)) + pos_est->attitude_estimation->linear_acc[i]  * dt;
	}
	
	// calculate velocity in global frame
	// vel = qe *vel_bf * qe - 1
	qvel_bf.s = 0.0f; qvel_bf.v[0] = pos_est->vel_bf[0]; qvel_bf.v[1] = pos_est->vel_bf[1]; qvel_bf.v[2] = pos_est->vel_bf[2];
	qvel = quaternions_local_to_global(pos_est->attitude_estimation->qe, qvel_bf);
	pos_est->vel[0] = qvel.v[0]; pos_est->vel[1] = qvel.v[1]; pos_est->vel[2] = qvel.v[2];
	
	for (i = 0; i < 3; i++)
	{
		// clean position estimate without gravity:
		//prev_pos[i] = localPosition.pos[i];
		pos_est->localPosition.pos[i] = pos_est->localPosition.pos[i] * (1.0f - (POS_DECAY * dt)) + pos_est->vel[i] * dt;
		pos_est->localPosition.heading = coord_conventions_get_yaw(pos_est->attitude_estimation->qe);
	}
	
}


static void position_estimation_position_correction(position_estimator_t *pos_est)
{
	global_position_t global_gps_position;
	local_coordinates_t local_coordinates;
	
	float dt = pos_est->attitude_estimation->dt;
	
	// UQuat_t bias_correction = {.s = 0, .v = {0.0f, 0.0f, 1.0f}};
	UQuat_t vel_correction = {.s = 0, .v = {0.0f, 0.0f, 1.0f}};
	float pos_error[3] = {0.0f,0.0f,0.0f};
	float baro_alt_error = 0.0f;
	float baro_vel_error = 0.0f;
	float baro_gain = 0.0f;
	float gps_gain = 0.0f;
	float gps_dt = 0.0f;
	float vel_error[3] = {0.0f,0.0f,0.0f};
	uint32_t tinterGps, tinterBaro;
	int32_t i;

	if (pos_est->init_barometer)
	{		
		// altimeter correction
		if (bmp085_newValidBarometer(pos_est->barometer, &pos_est->time_last_barometer_msg))
		{
			//alt_error = -(pos_est->barometer->altitude + pos_est->barometer->altitude_offset) - pos_est->localPosition.pos[2] + pos_est->localPosition.origin.altitude;
			pos_est->last_alt = -(pos_est->barometer->altitude ) + pos_est->localPosition.origin.altitude;
			baro_alt_error = -(pos_est->barometer->altitude ) - pos_est->localPosition.pos[2] + pos_est->localPosition.origin.altitude;
			/*print_util_dbg_print("alt_error:");
			print_util_dbg_print_num(pos_est->baro_alt_error,10);
			print_util_dbg_print(" = -(");
			print_util_dbg_print_num(pos_est->barometer->altitude,10);
			print_util_dbg_print(" + ");
			print_util_dbg_print_num(pos_est->barometer->altitude_offset,10);
			print_util_dbg_print(") - ");
			print_util_dbg_print_num(pos_est->localPosition.pos[2],10);
			print_util_dbg_print(" + ");
			print_util_dbg_print_num(pos_est->localPosition.origin.altitude,10);
			print_util_dbg_print("\n");*/
			pos_est->time_last_barometer_msg = pos_est->barometer->last_update;
		}
		tinterBaro = (time_keeper_get_micros() - pos_est->barometer->last_update) / 1000.0f;
		baro_gain = 1.0f; //math_util_fmax(1.0f - tinterBaro / 1000.0f, 0.0f);
			
		//pos_est->localPosition.pos[2] += kp_alt / ((float)(tinterBaro / 2.5f + 1.0f)) * alt_error;
		baro_alt_error = pos_est->last_alt  - pos_est->localPosition.pos[2];
		baro_vel_error = pos_est->barometer->vario_vz - pos_est->vel[2];
		//vel_error[2] = 0.1f * pos_error[2];
		//pos_est->vel[2] += kp_alt_v * vel_error[2];
				
	}
	else
	{
		barometer_offset_init(pos_est);
	}
	
	if (pos_est->init_gps_position)
	{
		if (gps_ublox_newValidGpsMsg(pos_est->gps, &pos_est->time_last_gps_msg))
		{
			global_gps_position.longitude = pos_est->gps->longitude;
			global_gps_position.latitude = pos_est->gps->latitude;
			global_gps_position.altitude = pos_est->gps->altitude;
			global_gps_position.heading = 0.0f;
			local_coordinates = coord_conventions_global_to_local_position(global_gps_position,pos_est->localPosition.origin);
			local_coordinates.timestamp_ms = pos_est->gps->time_last_msg;
			// compute GPS velocity estimate
			gps_dt = (local_coordinates.timestamp_ms - pos_est->lastGpsPos.timestamp_ms) / 1000.0f;
			if (gps_dt > 0.001f)
			{
				for (i = 0; i < 3; i++)
				{
					pos_est->last_vel[i] = (local_coordinates.pos[i] - pos_est->lastGpsPos.pos[i]) / gps_dt;
				}
				pos_est->lastGpsPos = local_coordinates;
			}
			else
			{
				print_util_dbg_print("GPS dt is too small!");
			}
		}
		tinterGps = time_keeper_get_millis() - pos_est->gps->time_last_msg;
			
		//gps_gain = math_util_fmax(1.0f - tinterGps / 1000.0f, 0.0f);
		gps_gain = 1.0f;
			
		for (i = 0;i < 3;i++)
		{
			pos_error[i] = pos_est->lastGpsPos.pos[i] - pos_est->localPosition.pos[i];
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
		pos_est->localPosition.pos[i] += pos_est->kp_pos[i] * gps_gain * pos_error[i]* dt;
	}
	pos_est->localPosition.pos[2] += pos_est->kp_alt * baro_gain * baro_alt_error* dt;


	for (i = 0; i < 3; i++)
	{
		vel_correction.v[i] = vel_error[i];
	}
				
	for (i = 0;i < 3;i++)
	{			
		pos_est->vel[i] += pos_est->kp_vel[i] * gps_gain * vel_correction.v[i]* dt;
	}
	pos_est->vel[2] += pos_est->kp_vel_baro * baro_gain * baro_vel_error* dt;
}

static void gps_position_init(position_estimator_t *pos_est)
{
	int32_t i;
	
	if (gps_ublox_newValidGpsMsg(pos_est->gps, &pos_est->time_last_gps_msg) && (!(pos_est->init_gps_position)))
	{
		pos_est->init_gps_position = true;
		
		pos_est->localPosition.origin.longitude = pos_est->gps->longitude;
		pos_est->localPosition.origin.latitude = pos_est->gps->latitude;
		pos_est->localPosition.origin.altitude = pos_est->gps->altitude;
		pos_est->localPosition.timestamp_ms = pos_est->gps->time_last_msg;

		pos_est->lastGpsPos = pos_est->localPosition;
		
		pos_est->last_alt = 0;
		for(i = 0;i < 3;i++)
		{
			pos_est->pos_correction[i] = 0.0f;
			pos_est->last_vel[i] = 0.0f;
			pos_est->localPosition.pos[i] = 0.0f;
			pos_est->vel[i] = 0.0f;
		}
		
		print_util_dbg_print("GPS position initialized!\n");
		
		// Resets the simulated position and velocities
		//*pos_est->sim_local_position = pos_est->localPosition;
	}
}


static void barometer_offset_init(position_estimator_t *pos_est)
{
	bool boolNewBaro = bmp085_newValidBarometer(pos_est->barometer, &pos_est->time_last_barometer_msg);

		
	//if ((centralData->init_gps_position)&&(boolNewBaro))
	if ((boolNewBaro))
	{
		
		pos_est->barometer->altitude_offset = -(pos_est->barometer->altitude - pos_est->localPosition.origin.altitude);
		//pos_est->barometer->altitude_offset = -pos_est->barometer->altitude - pos_est->localPosition.pos[2] + pos_est->localPosition.origin.altitude;
		pos_est->init_barometer = true;
		
		print_util_dbg_print("Offset of the barometer set to the GPS altitude, offset value of:");
		print_util_dbg_print_num(pos_est->barometer->altitude_offset,10);
		print_util_dbg_print(" = -");
		print_util_dbg_print_num(pos_est->barometer->altitude,10);
		print_util_dbg_print(" - ");
		print_util_dbg_print_num(pos_est->localPosition.pos[2],10);
		print_util_dbg_print(" + ");
		print_util_dbg_print_num(pos_est->localPosition.origin.altitude,10);
		print_util_dbg_print("\n");
	}
}


static void position_estimation_set_new_home_position(position_estimator_t *pos_est, mavlink_command_long_t* packet)
{
	if (packet->param1 == 1)
 	{
 		// Set new home position to actual position
 		print_util_dbg_print("Set new home location to actual position.\n");
 		pos_est->localPosition.origin = coord_conventions_local_to_global_position(pos_est->localPosition);
 		pos_est->sim_local_position->origin = pos_est->localPosition.origin;

 		print_util_dbg_print("New Home location: (");
 		print_util_dbg_print_num(pos_est->localPosition.origin.latitude * 10000000.0f,10);
 		print_util_dbg_print(", ");
 		print_util_dbg_print_num(pos_est->localPosition.origin.longitude * 10000000.0f,10);
 		print_util_dbg_print(", ");
 		print_util_dbg_print_num(pos_est->localPosition.origin.altitude * 1000.0f,10);
 		print_util_dbg_print(")\n");
 	}
 	else
 	{
 		// Set new home position from msg
 		print_util_dbg_print("Set new home location. \n");

 		pos_est->localPosition.origin.latitude = packet->param5;
 		pos_est->localPosition.origin.longitude = packet->param6;
 		pos_est->localPosition.origin.altitude = packet->param7;
		
		pos_est->sim_local_position->origin = pos_est->localPosition.origin;

 		print_util_dbg_print("New Home location: (");
 		print_util_dbg_print_num(pos_est->localPosition.origin.latitude * 10000000.0f,10);
 		print_util_dbg_print(", ");
 		print_util_dbg_print_num(pos_est->localPosition.origin.longitude * 10000000.0f,10);
 		print_util_dbg_print(", ");
 		print_util_dbg_print_num(pos_est->localPosition.origin.altitude * 1000.0f,10);
 		print_util_dbg_print(")\n");
 	}

	*pos_est->waypoint_set = false;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void position_estimation_init(position_estimator_t *pos_est, pressure_data_t *barometer, gps_Data_type_t *gps, AHRS_t *attitude_estimation, Imu_Data_t *imu, local_coordinates_t *sim_local_position, bool* waypoint_set, mavlink_communication_t *mavlink_communication, float home_lat, float home_lon, float home_alt, float gravity)
{
    int32_t i;

	pos_est->barometer = barometer;
	pos_est->gps = gps;
	pos_est->attitude_estimation = attitude_estimation;
	pos_est->imu = imu;
	pos_est->sim_local_position = sim_local_position;
	pos_est->waypoint_set = waypoint_set;
	
	// default GPS home position
	pos_est->localPosition.origin.longitude =   home_lon;
	pos_est->localPosition.origin.latitude =   home_lat;
	pos_est->localPosition.origin.altitude =   home_alt;
	pos_est->localPosition.pos[X] = 0;
	pos_est->localPosition.pos[Y] = 0;
	pos_est->localPosition.pos[Z] = 0;
	
    // reset position estimator
    pos_est->last_alt = 0;
    for(i = 0;i < 3;i++)
    {
        pos_est->pos_correction[i] = 0.0f;
        pos_est->last_vel[i] = 0.0f;
        pos_est->vel[i] = 0.0f;
        pos_est->vel_bf[i] = 0.0f;
    }

	pos_est->gravity = gravity;
	
	pos_est->init_gps_position = false;
	pos_est->init_barometer = false;
	pos_est->time_last_gps_msg = 0;
	pos_est->time_last_barometer_msg = 0;
	
    pos_est->kp_pos[X] = 2.0f;
    pos_est->kp_pos[Y] = 2.0f;
    pos_est->kp_pos[Z] = 1.0f;

    pos_est->kp_vel[X] = 1.0f;
    pos_est->kp_vel[Y] = 1.0f;
    pos_est->kp_vel[Z] = 0.5f;
	
	pos_est->kp_alt = 2.0f;
	pos_est->kp_vel_baro = 1.0f;
	
	gps_position_init(pos_est);
	
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	callbackcmd.command_id    = MAV_CMD_DO_SET_HOME; // 179
	callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER;
	callbackcmd.function      = (mavlink_cmd_callback_function_t)	&position_estimation_set_new_home_position;
	callbackcmd.module_struct = pos_est;
	mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);
	
	print_util_dbg_print("Position estimation initialized.\n");
}

void position_estimation_reset_home_altitude(position_estimator_t *pos_est)
{
	int32_t i;
	// reset origin to position where quad is armed if we have GPS
	if (pos_est->init_gps_position)
	{
		pos_est->localPosition.origin.longitude = pos_est->gps->longitude;
		pos_est->localPosition.origin.latitude = pos_est->gps->latitude;
		pos_est->localPosition.origin.altitude = pos_est->gps->altitude;
		pos_est->localPosition.timestamp_ms = pos_est->gps->time_last_msg;

		pos_est->lastGpsPos = pos_est->localPosition;
		
		// Resets the simulated position and velocities
		//*pos_est->sim_local_position = pos_est->localPosition;
			
	//}
	//else
	//{
		//pos_est->localPosition.origin.longitude = HOME_LONGITUDE;
		//pos_est->localPosition.origin.latitude = HOME_LATITUDE;
		//pos_est->localPosition.origin.altitude = HOME_ALTITUDE;
		//
		//pos_est->simLocalPos->origin.longitude = HOME_LONGITUDE;
		//pos_est->simLocalPos->origin.latitude = HOME_LATITUDE;
		//pos_est->simLocalPos->origin.altitude = HOME_ALTITUDE;
		//
	}
	// reset barometer offset
	pos_est->barometer->altitude_offset = -(pos_est->barometer->altitude - pos_est->barometer->altitude_offset - pos_est->localPosition.origin.altitude);
	//pos_est->barometer->altitude_offset = -pos_est->barometer->altitude - pos_est->localPosition.pos[2] + pos_est->localPosition.origin.altitude;
	pos_est->init_barometer = true;
		
	print_util_dbg_print("Offset of the barometer set to the GPS altitude, offset value of:");
	print_util_dbg_print_num(pos_est->barometer->altitude_offset,10);
	print_util_dbg_print(" = -");
	print_util_dbg_print_num(pos_est->barometer->altitude,10);
	print_util_dbg_print(" - ");
	print_util_dbg_print_num(pos_est->localPosition.pos[2],10);
	print_util_dbg_print(" + ");
	print_util_dbg_print_num(pos_est->localPosition.origin.altitude,10);
	print_util_dbg_print("\n");

	// reset position estimator
	pos_est->last_alt = 0;
	for(i = 0;i < 3;i++)
	{
		pos_est->pos_correction[i] = 0.0f;
		pos_est->last_vel[i] = 0.0f;
		pos_est->localPosition.pos[i] = 0.0f;
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
