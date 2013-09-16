/*
* position_estimation.c
*
* Created: 05.09.2013 17:47:51
*  Author: ndousse
*/


#include "position_estimation.h"
#include "boardsupport.h"
#include "qfilter.h"
#include "coord_conventions.h"
#include "gps_ublox.h"
#include "print_util.h"
#include "bmp085.h"
#include "math_util.h"

board_hardware_t *board;



float kp_pos[3],kp_vel[3], kp_alt, kp_alt_v;

uint32_t timeLastGpsMsg;
uint32_t timeLastBarometerMsg;

void init_pos_integration()
{
	board = get_board_hardware();
	kp_alt=1.0;
	kp_alt_v=0.1;
	board->init_gps_position = false;
	board->init_barometer=false;
	timeLastGpsMsg = 0;
	timeLastBarometerMsg=get_micros();
	
	kp_pos[0] = 1.5;
	kp_pos[1] = 1.5;
	kp_pos[2] = 1.0;
	
	kp_vel[0] = 0.5;
	kp_vel[1] = 0.5;
	kp_vel[2] = 0.4;
	
	init_pos_gps();
}

void init_pos_gps()
{
	int i;
	
	if (newValidGpsMsg(&timeLastGpsMsg) && (!(board->init_gps_position)))
	{
		board->init_gps_position = true;
		
		board->imu1.attitude.localPosition.origin.longitude = board->GPS_data.longitude;
		board->imu1.attitude.localPosition.origin.latitude = board->GPS_data.latitude;
		board->imu1.attitude.localPosition.origin.altitude = board->GPS_data.altitude;
		board->imu1.attitude.localPosition.timestamp_ms=board->GPS_data.timeLastMsg;

		board->imu1.attitude.lastGpsPos=board->imu1.attitude.localPosition;
		
		board->imu1.attitude.baro_alt_error=0.0;
		board->imu1.attitude.last_alt=0;
		for(i=0;i<3;i++)
		{
			board->imu1.attitude.pos_correction[i]=0.0;
			board->imu1.attitude.last_vel[i]=0.0;
			board->imu1.attitude.localPosition.pos[i] = 0.0;
			board->imu1.attitude.vel[i]=0.0;
		}
		
		dbg_print("GPS position initialized!\n");
	}
}

void init_barometer_offset()
{
	bool boolNewBaro = newValidBarometer(&timeLastBarometerMsg);
	dbg_print("boolNewBaro:");
	dbg_print_num(boolNewBaro,10);
	dbg_print("\n");

		
	//if ((board->init_gps_position)&&(boolNewBaro))
	if ((boolNewBaro))
	{
		
		board->pressure.altitude_offset = -(board->pressure.altitude - board->imu1.attitude.localPosition.origin.altitude);
		//board->pressure.altitude_offset = -board->pressure.altitude - board->imu1.attitude.localPosition.pos[2] + board->imu1.attitude.localPosition.origin.altitude;
		board->init_barometer = true;
		
		dbg_print("Offset of the barometer set to the GPS altitude, offset value of:");
		dbg_print_num(board->pressure.altitude_offset,10);
		dbg_print(" = -");
		dbg_print_num(board->pressure.altitude,10);
		dbg_print(" - ");
		dbg_print_num(board->imu1.attitude.localPosition.pos[2],10);
		dbg_print(" + ");
		dbg_print_num(board->imu1.attitude.localPosition.origin.altitude,10);
		dbg_print("\n");
	}
}

void position_integration(Quat_Attitude_t *attitude, float dt)
{
	int i;
	
	UQuat_t qvel_bf,qvel, qtmp1, qtmp2, qtmp3;
	float tmp[3];
	
	for (i=0; i<3; i++) {
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
	}
}

void position_correction()
{
	global_position_t global_gps_position;
	local_coordinates_t local_coordinates;
	
	UQuat_t bias_correction ={.s=0, .v={0.0, 0.0, 1.0}};
	UQuat_t vel_correction ={.s=0, .v={0.0, 0.0, 1.0}};
	float pos_error[3]= {0.0,0.0,0.0};
	float baro_gain=0.0;
	float gps_gain=0.0;
	
	float vel_error[3]={0.0,0.0,0.0};
	uint32_t tinterGps, tinterBaro;
	int i;
	float dt;
	if ((board->simulation_mode == 0))
	{
		if (board->init_barometer)
		{
			// altimeter correction
			if (newValidBarometer(&timeLastBarometerMsg))
			{
				//alt_error = -(board->pressure.altitude + board->pressure.altitude_offset) - board->imu1.attitude.localPosition.pos[2]+board->imu1.attitude.localPosition.origin.altitude;
				board->imu1.attitude.last_alt= -(board->pressure.altitude + board->pressure.altitude_offset) + board->imu1.attitude.localPosition.origin.altitude;
				board->imu1.attitude.baro_alt_error = -(board->pressure.altitude + board->pressure.altitude_offset) - board->imu1.attitude.localPosition.pos[2]+board->imu1.attitude.localPosition.origin.altitude;
				dbg_print("alt_error:");
				dbg_print_num(board->imu1.attitude.baro_alt_error,10);
				dbg_print(" = -(");
				dbg_print_num(board->pressure.altitude,10);
				dbg_print(" + ");
				dbg_print_num(board->pressure.altitude_offset,10);
				dbg_print(") - ");
				dbg_print_num(board->imu1.attitude.localPosition.pos[2],10);
				dbg_print(" + ");
				dbg_print_num(board->imu1.attitude.localPosition.origin.altitude,10);
				dbg_print("\n");
				timeLastBarometerMsg=board->pressure.last_update;
			}
			tinterBaro = (get_micros()-board->pressure.last_update)/1000.0;
			baro_gain=fmax(1.0-tinterBaro/1000.0, 0.0);
			
			//board->imu1.attitude.localPosition.pos[2] += kp_alt/((float)(tinterBaro/2.5 + 1.0)) * alt_error;
			pos_error[2]=board->imu1.attitude.last_alt  - board->imu1.attitude.localPosition.pos[2];
			vel_error[2]=board->pressure.vario_vz - board->imu1.attitude.vel[2];
			//board->imu1.attitude.vel[2] += kp_alt_v * vel_error[2];
				
		}else{
			init_barometer_offset();
		}
	
		if (board->init_gps_position)
		{
			if (newValidGpsMsg(&timeLastGpsMsg))
			{
				//dbg_print("New valid message\n");
				global_gps_position.longitude = board->GPS_data.longitude;
				global_gps_position.latitude = board->GPS_data.latitude;
				global_gps_position.altitude = board->GPS_data.altitude;
			
				local_coordinates = global_to_local_position(global_gps_position,board->imu1.attitude.localPosition.origin);
				local_coordinates.timestamp_ms=board->GPS_data.timeLastMsg;
				// compute GPS velocity estimate
				dt=(local_coordinates.timestamp_ms - board->imu1.attitude.lastGpsPos.timestamp_ms)/1000.0;
				for (i=0; i<2; i++) board->imu1.attitude.last_vel[i] = (local_coordinates.pos[i]-board->imu1.attitude.lastGpsPos.pos[i])/dt;
				board->imu1.attitude.lastGpsPos=local_coordinates;
			}
			tinterGps = get_millis() - board->GPS_data.timeLastMsg;
			
			gps_gain=fmax(1.0-tinterGps/1000.0, 0.0);
			gps_gain=1.0;
			
			for (i=0;i<2;i++){
				pos_error[i] = board->imu1.attitude.lastGpsPos.pos[i] - board->imu1.attitude.localPosition.pos[i];
				vel_error[i] = board->imu1.attitude.last_vel[i]       - board->imu1.attitude.vel[i]; 
			}
				
		
		}else{
			init_pos_gps();
			for (i=0;i<2;i++){
				pos_error[i] = board->imu1.attitude.lastGpsPos.pos[i] - board->imu1.attitude.localPosition.pos[i];
				vel_error[i] = 0.0;
			}
			gps_gain=0.1;
		}
		
		// Apply error correction to velocity and position estimates
		for (i=0;i<2;i++) {
			board->imu1.attitude.localPosition.pos[i] += kp_pos[i] * gps_gain * pos_error[i]* board->imu1.dt;
		}
		board->imu1.attitude.localPosition.pos[2] += kp_alt * baro_gain * pos_error[2]* board->imu1.dt;
		
		for (i=0; i<3; i++) vel_correction.v[i] = vel_error[i];
		vel_correction = quat_global_to_local(board->imu1.attitude.qe, vel_correction);
				
		for (i=0;i<3;i++) {
			board->imu1.attitude.vel_bf[i] += kp_vel[i]*gps_gain * vel_correction.v[i]* board->imu1.dt;
			//board->imu1.attitude.be[i+ACC_OFFSET] -=  0.01* kp_vel[i]*gps_gain *  vel_correction.v[i]* board->imu1.dt;
		}

	}
}