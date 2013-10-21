/*
* position_estimation.c
*
* Created: 05.09.2013 17:47:51
*  Author: ndousse
*/


#include "position_estimation.h"
#include "central_data.h"
#include "qfilter.h"
#include "coord_conventions.h"
#include "gps_ublox.h"
#include "print_util.h"
#include "bmp085.h"
#include "math_util.h"
#include "time_keeper.h"

central_data_t *centralData;


float kp_pos[3],kp_vel[3], kp_alt;

uint32_t timeLastGpsMsg;
uint32_t timeLastBarometerMsg;

void init_pos_integration()
{
	centralData = get_central_data();
	kp_alt=0.4;
	centralData->init_gps_position = false;
	centralData->init_barometer=false;
	timeLastGpsMsg = 0;
	timeLastBarometerMsg=get_micros();
	
	kp_pos[0] = 1.0;
	kp_pos[1] = 1.0;
	kp_pos[2] = 2.0;
	
	kp_vel[0] = 0.5;
	kp_vel[1] = 0.5;
	kp_vel[2] = 0.7;
	
	init_pos_gps();
}

void init_pos_gps()
{
	int i;
	
	if (newValidGpsMsg(&timeLastGpsMsg) && (!(centralData->init_gps_position)))
	{
		centralData->init_gps_position = true;
		
		centralData->imu1.attitude.localPosition.origin.longitude = centralData->GPS_data.longitude;
		centralData->imu1.attitude.localPosition.origin.latitude = centralData->GPS_data.latitude;
		centralData->imu1.attitude.localPosition.origin.altitude = centralData->GPS_data.altitude;
		centralData->imu1.attitude.localPosition.timestamp_ms=centralData->GPS_data.timeLastMsg;

		centralData->imu1.attitude.lastGpsPos=centralData->imu1.attitude.localPosition;
		
		centralData->imu1.attitude.baro_alt_error=0.0;
		centralData->imu1.attitude.last_alt=0;
		for(i=0;i<3;i++)
		{
			centralData->imu1.attitude.pos_correction[i]=0.0;
			centralData->imu1.attitude.last_vel[i]=0.0;
			centralData->imu1.attitude.localPosition.pos[i] = 0.0;
			centralData->imu1.attitude.vel[i]=0.0;
		}
		
		dbg_print("GPS position initialized!\n");
	}
}

void init_barometer_offset()
{
	bool boolNewBaro = newValidBarometer(&timeLastBarometerMsg);

		
	//if ((centralData->init_gps_position)&&(boolNewBaro))
	if ((boolNewBaro))
	{
		
		centralData->pressure.altitude_offset = -(centralData->pressure.altitude - centralData->imu1.attitude.localPosition.origin.altitude);
		//centralData->pressure.altitude_offset = -centralData->pressure.altitude - centralData->imu1.attitude.localPosition.pos[2] + centralData->imu1.attitude.localPosition.origin.altitude;
		centralData->init_barometer = true;
		
		dbg_print("Offset of the barometer set to the GPS altitude, offset value of:");
		dbg_print_num(centralData->pressure.altitude_offset,10);
		dbg_print(" = -");
		dbg_print_num(centralData->pressure.altitude,10);
		dbg_print(" - ");
		dbg_print_num(centralData->imu1.attitude.localPosition.pos[2],10);
		dbg_print(" + ");
		dbg_print_num(centralData->imu1.attitude.localPosition.origin.altitude,10);
		dbg_print("\n");
	}
}

void position_integration(Quat_Attitude_t *attitude, float dt)
{
	int i;
	
	UQuat_t qvel_bf,qvel, qtmp1, qtmp2, qtmp3;
	float tmp[3];
	qvel.s=0;
	for (i=0; i<3; i++) qvel.v[i]=attitude->vel[i];
	qvel_bf=quat_global_to_local(attitude->qe, qvel);
	for (i=0; i<3; i++) {
		attitude->vel_bf[i]=qvel_bf.v[i];
		// clean acceleration estimate without gravity:
		attitude->acc_bf[i]=(attitude->a[i] - attitude->up_vec.v[i]) * GRAVITY;
		attitude->vel_bf[i]=attitude->vel_bf[i]*(1.0-(VEL_DECAY*dt)) + attitude->acc_bf[i] * dt;
	}
	
	// calculate velocity in global frame
	// vel = qe *vel_bf * qe-1
	qvel_bf.s= 0.0; qvel_bf.v[0]=attitude->vel_bf[0]; qvel_bf.v[1]=attitude->vel_bf[1]; qvel_bf.v[2]=attitude->vel_bf[2];
	qvel = quat_local_to_global(attitude->qe, qvel_bf);
	attitude->vel[0]=qvel.v[0]; attitude->vel[1]=qvel.v[1]; attitude->vel[2]=qvel.v[2];
	
	// calculate velocity in global frame
	// vel = qe *vel_bf * qe-1
	//qtmp1.s= 0.0; qtmp1.v[0]=attitude->vel_bf[0]; qtmp1.v[1]=attitude->vel_bf[1]; qtmp1.v[2]=attitude->vel_bf[2];
	//QMUL(attitude->qe, qtmp1, qtmp2);
	//QI(attitude->qe, qtmp1);
	//QMUL(qtmp2, qtmp1, qtmp3);
	//attitude->vel[0]=qtmp3.v[0]; attitude->vel[1]=qtmp3.v[1]; attitude->vel[2]=qtmp3.v[2];
	
	for (i=0; i<3; i++) {
		// clean position estimate without gravity:
		attitude->localPosition.pos[i] =attitude->localPosition.pos[i]*(1.0-(POS_DECAY*dt)) + attitude->vel[i] *dt;
		attitude->localPosition.heading=get_yaw(attitude->qe);
	}
}

void position_correction()
{
	global_position_t global_gps_position;
	local_coordinates_t local_coordinates;
	
	UQuat_t bias_correction ={.s=0, .v={0.0, 0.0, 1.0}};
	UQuat_t vel_correction ={.s=0, .v={0.0, 0.0, 1.0}};
	float pos_error[3]= {0.0,0.0,0.0};
	float prev_pos[]={0.0,0.0,0.0};
	float baro_gain=0.0;
	float gps_gain=0.0;
	
	float vel_error[3]={0.0,0.0,0.0};
	uint32_t tinterGps, tinterBaro;
	int i;
	float dt;
	//if ((centralData->simulation_mode == 0))
	{
		if (centralData->init_barometer)
		{
			// altimeter correction
			if (newValidBarometer(&timeLastBarometerMsg))
			{
				//alt_error = -(centralData->pressure.altitude + centralData->pressure.altitude_offset) - centralData->imu1.attitude.localPosition.pos[2]+centralData->imu1.attitude.localPosition.origin.altitude;
				centralData->imu1.attitude.last_alt= -(centralData->pressure.altitude + centralData->pressure.altitude_offset) + centralData->imu1.attitude.localPosition.origin.altitude;
				centralData->imu1.attitude.baro_alt_error = -(centralData->pressure.altitude + centralData->pressure.altitude_offset) - centralData->imu1.attitude.localPosition.pos[2]+centralData->imu1.attitude.localPosition.origin.altitude;
				/*dbg_print("alt_error:");
				dbg_print_num(centralData->imu1.attitude.baro_alt_error,10);
				dbg_print(" = -(");
				dbg_print_num(centralData->pressure.altitude,10);
				dbg_print(" + ");
				dbg_print_num(centralData->pressure.altitude_offset,10);
				dbg_print(") - ");
				dbg_print_num(centralData->imu1.attitude.localPosition.pos[2],10);
				dbg_print(" + ");
				dbg_print_num(centralData->imu1.attitude.localPosition.origin.altitude,10);
				dbg_print("\n");*/
				timeLastBarometerMsg=centralData->pressure.last_update;
			}
			tinterBaro = (get_micros()-centralData->pressure.last_update)/1000.0;
			baro_gain=fmax(1.0-tinterBaro/1000.0, 0.0);
			
			//centralData->imu1.attitude.localPosition.pos[2] += kp_alt/((float)(tinterBaro/2.5 + 1.0)) * alt_error;
			pos_error[2]=centralData->imu1.attitude.last_alt  - centralData->imu1.attitude.localPosition.pos[2];
			vel_error[2]=centralData->pressure.vario_vz - centralData->imu1.attitude.vel[2];
			//vel_error[2]=0.1*pos_error[2];
			//centralData->imu1.attitude.vel[2] += kp_alt_v * vel_error[2];
				
		}else{
			init_barometer_offset();
		}
	
		if (centralData->init_gps_position)
		{
			if (newValidGpsMsg(&timeLastGpsMsg))
			{
				//dbg_print("New valid message\n");
				global_gps_position.longitude = centralData->GPS_data.longitude;
				global_gps_position.latitude = centralData->GPS_data.latitude;
				global_gps_position.altitude = centralData->GPS_data.altitude;
			
				local_coordinates = global_to_local_position(global_gps_position,centralData->imu1.attitude.localPosition.origin);
				local_coordinates.timestamp_ms=centralData->GPS_data.timeLastMsg;
				// compute GPS velocity estimate
				dt=(local_coordinates.timestamp_ms - centralData->imu1.attitude.lastGpsPos.timestamp_ms)/1000.0;
				for (i=0; i<2; i++) centralData->imu1.attitude.last_vel[i] = (local_coordinates.pos[i]-centralData->imu1.attitude.lastGpsPos.pos[i])/dt;
				centralData->imu1.attitude.lastGpsPos=local_coordinates;
			}
			tinterGps = get_millis() - centralData->GPS_data.timeLastMsg;
			
			gps_gain=fmax(1.0-tinterGps/1000.0, 0.0);
			gps_gain=1.0;
			
			for (i=0;i<2;i++){
				pos_error[i] = centralData->imu1.attitude.lastGpsPos.pos[i] - centralData->imu1.attitude.localPosition.pos[i];
				vel_error[i] = centralData->imu1.attitude.last_vel[i]       - centralData->imu1.attitude.vel[i]; 
			}
				
		
		}else{
			init_pos_gps();
			for (i=0;i<2;i++){
				//pos_error[i] = centralData->imu1.attitude.lastGpsPos.pos[i] - centralData->imu1.attitude.localPosition.pos[i];
				pos_error[i] = 0.0;
				vel_error[i] = 0.0;
			}
			gps_gain=0.1;
		}
		
		for (i=0;i<3;i++) {
			prev_pos[i]=centralData->imu1.attitude.localPosition.pos[i];
		}
		// Apply error correction to velocity and position estimates
		for (i=0;i<2;i++) {
			centralData->imu1.attitude.localPosition.pos[i] += kp_pos[i] * gps_gain * pos_error[i]* centralData->imu1.dt;
		}
		centralData->imu1.attitude.localPosition.pos[2] += kp_alt * baro_gain * pos_error[2]* centralData->imu1.dt;
		
		for (i=0; i<3; i++) vel_correction.v[i] = vel_error[i];
		//for (i=0; i<3; i++) {
		//	centralData->imu1.attitude.vel[i] = 0.99*(centralData->imu1.attitude.vel[i]) + 0.01*(centralData->imu1.attitude.localPosition.pos[i] - prev_pos[i])/centralData->imu1.dt;
		//}
		//vel_correction = quat_global_to_local(centralData->imu1.attitude.qe, vel_correction);
				
		for (i=0;i<2;i++) {			
			centralData->imu1.attitude.vel[i] += kp_vel[i]*gps_gain * vel_correction.v[i]* centralData->imu1.dt;
		}
		centralData->imu1.attitude.vel[2] += kp_vel[2]*baro_gain * vel_correction.v[2]* centralData->imu1.dt;

	}
}