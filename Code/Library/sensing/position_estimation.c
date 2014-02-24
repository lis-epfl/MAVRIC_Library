/*
* position_estimation.c
*
* Created: 05.09.2013 17:47:51
*  Author: ndousse
*/


#include "position_estimation.h"
#include "qfilter.h"
#include "coord_conventions.h"
#include "gps_ublox.h"
#include "print_util.h"
#include "bmp085.h"
#include "math_util.h"
#include "time_keeper.h"

//central_data_t *centralData;

//float prev_pos[3];

void init_pos_gps(position_estimator_t *pos_est, gps_Data_type *gps);
void init_barometer_offset(position_estimator_t *pos_est, pressure_data *barometer);


void init_pos_integration(position_estimator_t *pos_est, pressure_data *barometer, gps_Data_type *gps)
{
	//centralData = get_central_data();
	pos_est->init_gps_position = false;
	pos_est->init_barometer=false;
	pos_est->timeLastGpsMsg = 0;
	pos_est->timeLastBarometerMsg=0;
	
	pos_est->kp_pos[0] = 2.0;
	pos_est->kp_pos[1] = 2.0;
	pos_est->kp_pos[2] = 1.0;
	
	pos_est->kp_vel[0] = 1.0;
	pos_est->kp_vel[1] = 1.0;
	pos_est->kp_vel[2] = 0.5;
	
	pos_est->kp_alt=2.0;
	pos_est->kp_vel_baro=1.0;
	
	init_pos_gps(pos_est, gps);
}

void init_pos_gps(position_estimator_t *pos_est, gps_Data_type *gps)
{
	int i;
	
	if (newValidGpsMsg(&pos_est->timeLastGpsMsg) && (!(pos_est->init_gps_position)))
	{
		pos_est->init_gps_position = true;
		
		pos_est->localPosition.origin.longitude = gps->longitude;
		pos_est->localPosition.origin.latitude = gps->latitude;
		pos_est->localPosition.origin.altitude = gps->altitude;
		pos_est->localPosition.timestamp_ms=gps->timeLastMsg;

		pos_est->lastGpsPos=pos_est->localPosition;
		
		
		pos_est->last_alt=0;
		for(i=0;i<3;i++)
		{
			pos_est->pos_correction[i]=0.0;
			pos_est->last_vel[i]=0.0;
			pos_est->localPosition.pos[i] = 0.0;
			pos_est->vel[i]=0.0;
		}
		
		dbg_print("GPS position initialized!\n");
	}
}

void init_barometer_offset(position_estimator_t *pos_est, pressure_data *barometer)
{
	bool boolNewBaro = newValidBarometer(&pos_est->timeLastBarometerMsg);

		
	//if ((centralData->init_gps_position)&&(boolNewBaro))
	if ((boolNewBaro))
	{
		
		barometer->altitude_offset = -(barometer->altitude - pos_est->localPosition.origin.altitude);
		//barometer->altitude_offset = -barometer->altitude - pos_est->localPosition.pos[2] + pos_est->localPosition.origin.altitude;
		pos_est->init_barometer = true;
		
		dbg_print("Offset of the barometer set to the GPS altitude, offset value of:");
		dbg_print_num(barometer->altitude_offset,10);
		dbg_print(" = -");
		dbg_print_num(barometer->altitude,10);
		dbg_print(" - ");
		dbg_print_num(pos_est->localPosition.pos[2],10);
		dbg_print(" + ");
		dbg_print_num(pos_est->localPosition.origin.altitude,10);
		dbg_print("\n");
	}
}

void position_reset_home_altitude(position_estimator_t *pos_est, pressure_data *barometer, gps_Data_type *gps) {
		int i;
		// reset origin to position where quad is armed if we have GPS
		if (pos_est->init_gps_position) {
			pos_est->localPosition.origin.longitude = gps->longitude;
			pos_est->localPosition.origin.latitude = gps->latitude;
			pos_est->localPosition.origin.altitude = gps->altitude;
			pos_est->localPosition.timestamp_ms=gps->timeLastMsg;

			pos_est->lastGpsPos=pos_est->localPosition;
		}
		// reset barometer offset
		barometer->altitude_offset = -(barometer->altitude - barometer->altitude_offset - pos_est->localPosition.origin.altitude);
		//barometer->altitude_offset = -barometer->altitude - pos_est->localPosition.pos[2] + pos_est->localPosition.origin.altitude;
		pos_est->init_barometer = true;
		
		dbg_print("Offset of the barometer set to the GPS altitude, offset value of:");
		dbg_print_num(barometer->altitude_offset,10);
		dbg_print(" = -");
		dbg_print_num(barometer->altitude,10);
		dbg_print(" - ");
		dbg_print_num(pos_est->localPosition.pos[2],10);
		dbg_print(" + ");
		dbg_print_num(pos_est->localPosition.origin.altitude,10);
		dbg_print("\n");

		// reset position estimator
		pos_est->last_alt=0;
		for(i=0;i<3;i++)
		{
			pos_est->pos_correction[i]=0.0;
			pos_est->last_vel[i]=0.0;
			pos_est->localPosition.pos[i] = 0.0;
			pos_est->vel[i]=0.0;
		}

	
}

void position_integration(position_estimator_t *pos_est, Quat_Attitude_t *attitude, float dt)
{
	int i;
	
	UQuat_t qvel_bf,qvel; 
	// UQuat_t qtmp1, qtmp2, qtmp3;
	// float tmp[3];
	qvel.s=0;
	for (i=0; i<3; i++) qvel.v[i]=pos_est->vel[i];
	qvel_bf=quat_global_to_local(attitude->qe, qvel);
	for (i=0; i<3; i++) {
		pos_est->vel_bf[i]=qvel_bf.v[i];
		// clean acceleration estimate without gravity:
		attitude->acc_bf[i]=GRAVITY * (attitude->a[i] - attitude->up_vec.v[i]) ;
		pos_est->vel_bf[i]=pos_est->vel_bf[i]*(1.0-(VEL_DECAY*dt)) + attitude->acc_bf[i]  * dt;
	}
	
	// calculate velocity in global frame
	// vel = qe *vel_bf * qe-1
	qvel_bf.s= 0.0; qvel_bf.v[0]=pos_est->vel_bf[0]; qvel_bf.v[1]=pos_est->vel_bf[1]; qvel_bf.v[2]=pos_est->vel_bf[2];
	qvel = quat_local_to_global(attitude->qe, qvel_bf);
	pos_est->vel[0]=qvel.v[0]; pos_est->vel[1]=qvel.v[1]; pos_est->vel[2]=qvel.v[2];
	
	// calculate velocity in global frame
	// vel = qe *vel_bf * qe-1
	//qtmp1.s= 0.0; qtmp1.v[0]=attitude->vel_bf[0]; qtmp1.v[1]=attitude->vel_bf[1]; qtmp1.v[2]=attitude->vel_bf[2];
	//QMUL(attitude->qe, qtmp1, qtmp2);
	//QI(attitude->qe, qtmp1);
	//QMUL(qtmp2, qtmp1, qtmp3);
	//attitude->vel[0]=qtmp3.v[0]; attitude->vel[1]=qtmp3.v[1]; attitude->vel[2]=qtmp3.v[2];
	
	for (i=0; i<3; i++) {
		// clean position estimate without gravity:
		//prev_pos[i]=attitude->localPosition.pos[i];
		pos_est->localPosition.pos[i] =pos_est->localPosition.pos[i]*(1.0-(POS_DECAY*dt)) + pos_est->vel[i] *dt;
		pos_est->localPosition.heading=get_yaw(attitude->qe);
	}

}
	
void position_correction(position_estimator_t *pos_est, pressure_data *barometer, gps_Data_type *gps, float dt)
{
	global_position_t global_gps_position;
	local_coordinates_t local_coordinates;
	
	// UQuat_t bias_correction ={.s=0, .v={0.0, 0.0, 1.0}};
	UQuat_t vel_correction ={.s=0, .v={0.0, 0.0, 1.0}};
	float pos_error[3]= {0.0,0.0,0.0};
	float baro_alt_error=0.0;
	float baro_vel_error=0.0;
	float baro_gain=0.0;
	float gps_gain=0.0;
	float gps_dt=0.0;
	float vel_error[3]={0.0,0.0,0.0};
	uint32_t tinterGps, tinterBaro;
	int i;
	//if ((centralData->simulation_mode == 0))
	{
		if (pos_est->init_barometer)
		{
			// altimeter correction
			if (newValidBarometer(&pos_est->timeLastBarometerMsg))
			{
				//alt_error = -(barometer->altitude + barometer->altitude_offset) - pos_est->localPosition.pos[2]+pos_est->localPosition.origin.altitude;
				pos_est->last_alt= -(barometer->altitude ) + pos_est->localPosition.origin.altitude;
				baro_alt_error = -(barometer->altitude ) - pos_est->localPosition.pos[2]+pos_est->localPosition.origin.altitude;
				/*dbg_print("alt_error:");
				dbg_print_num(pos_est->baro_alt_error,10);
				dbg_print(" = -(");
				dbg_print_num(barometer->altitude,10);
				dbg_print(" + ");
				dbg_print_num(barometer->altitude_offset,10);
				dbg_print(") - ");
				dbg_print_num(pos_est->localPosition.pos[2],10);
				dbg_print(" + ");
				dbg_print_num(pos_est->localPosition.origin.altitude,10);
				dbg_print("\n");*/
				pos_est->timeLastBarometerMsg=barometer->last_update;
			}
			tinterBaro = (get_micros()-barometer->last_update)/1000.0;
			baro_gain=1.0;//fmax(1.0-tinterBaro/1000.0, 0.0);
			
			//pos_est->localPosition.pos[2] += kp_alt/((float)(tinterBaro/2.5 + 1.0)) * alt_error;
			baro_alt_error=pos_est->last_alt  - pos_est->localPosition.pos[2];
			baro_vel_error=barometer->vario_vz - pos_est->vel[2];
			//vel_error[2]=0.1*pos_error[2];
			//pos_est->vel[2] += kp_alt_v * vel_error[2];
				
		}else{
			init_barometer_offset(pos_est, barometer);
		}
	
		if (pos_est->init_gps_position)
		{
			if (newValidGpsMsg(&pos_est->timeLastGpsMsg))
			{
				//dbg_print("New valid message\n");
				global_gps_position.longitude = gps->longitude;
				global_gps_position.latitude = gps->latitude;
				global_gps_position.altitude = gps->altitude;
				global_gps_position.heading=0.0;
				local_coordinates = global_to_local_position(global_gps_position,pos_est->localPosition.origin);
				local_coordinates.timestamp_ms=gps->timeLastMsg;
				// compute GPS velocity estimate
				gps_dt=(local_coordinates.timestamp_ms - pos_est->lastGpsPos.timestamp_ms)/1000.0;
				if (gps_dt>0.001) {
					for (i=0; i<3; i++) pos_est->last_vel[i] = (local_coordinates.pos[i]-pos_est->lastGpsPos.pos[i])/gps_dt;
					pos_est->lastGpsPos=local_coordinates;
				} else dbg_print("GPS dt is too small!");
			}
			tinterGps = get_millis() - gps->timeLastMsg;
			
			gps_gain=fmax(1.0-tinterGps/1000.0, 0.0);
			gps_gain=1.0;
			
			for (i=0;i<3;i++){
				pos_error[i] = pos_est->lastGpsPos.pos[i] - pos_est->localPosition.pos[i];
				vel_error[i] = pos_est->last_vel[i]       - pos_est->vel[i]; 
			}
		}else{
			init_pos_gps(pos_est, gps);
			for (i=0;i<2;i++){
				//pos_error[i] = pos_est->lastGpsPos.pos[i] - pos_est->localPosition.pos[i];
				pos_error[i] = 0.0;
				vel_error[i] = 0.0;
			}
			gps_gain=0.1;
		}
		
		// Apply error correction to velocity and position estimates
		for (i=0;i<3;i++) {
			pos_est->localPosition.pos[i] += pos_est->kp_pos[i] * gps_gain * pos_error[i]* dt;
		}
		pos_est->localPosition.pos[2] += pos_est->kp_alt * baro_gain * baro_alt_error* dt;


		for (i=0; i<3; i++) vel_correction.v[i] = vel_error[i];
		//for (i=0; i<3; i++) vel_correction.v[i] = pos_error[i];
		//vel_correction = quat_global_to_local(pos_est->qe, vel_correction);
				
		for (i=0;i<3;i++) {			
			pos_est->vel[i] += pos_est->kp_vel[i]*gps_gain * vel_correction.v[i]* dt;
		}
		pos_est->vel[2] += pos_est->kp_vel_baro * baro_gain * baro_vel_error* dt;

	}
}