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

board_hardware_t *board;

global_position_t global_gps_position;
local_coordinates_t local_coordinates;
float alt_error;

float pos_err_prev;

float kp_pos[3],kp_vel[3], kp_alt, kp_alt_v;

uint32_t timeLastGpsMsg;
uint32_t timeLastBarometerMsg;

void init_pos_integration()
{
	board = get_board_hardware();
	kp_alt=0.005;
	kp_alt_v=0.001;
	board->init_gps_position = false;
	
	timeLastGpsMsg = 0;
	
	kp_pos[0] = 0.1;
	kp_pos[1] = 0.1;
	kp_pos[2] = 0.1;
	
	kp_vel[0] = 0.05;
	kp_vel[1] = 0.05;
	kp_vel[2] = 0.1;
	
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
		
		global_gps_position.longitude = board->GPS_data.longitude;
		global_gps_position.latitude = board->GPS_data.latitude;
		global_gps_position.altitude = board->GPS_data.altitude;
		
		local_coordinates = global_to_local_position(global_gps_position,board->imu1.attitude.localPosition.origin);
		
		for(i=0;i<3;i++)
		{
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
	
	if ((board->init_gps_position)&&(boolNewBaro))
	{
		board->pressure.altitude_offset = -board->pressure.altitude - board->imu1.attitude.localPosition.pos[2] + board->imu1.attitude.localPosition.origin.altitude;
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
	UQuat_t bias_correction ={.s=0, .v={0.0, 0.0, 1.0}};
	UQuat_t vel_correction ={.s=0, .v={0.0, 0.0, 1.0}};
	float pos_error[3];
	
	float vel_error[3];
	uint32_t tinterGps, tinterBaro;
	int i;
	if ((board->simulation_mode == 0))
	{
		if (board->init_barometer)
		{
			// altimeter correction
			if (newValidBarometer(&timeLastBarometerMsg))
			{
				//alt_error = -(board->pressure.altitude + board->pressure.altitude_offset) - board->imu1.attitude.localPosition.pos[2]+board->imu1.attitude.localPosition.origin.altitude;
				alt_error = -(board->altitude_filtered + board->pressure.altitude_offset) - board->imu1.attitude.localPosition.pos[2]+board->imu1.attitude.localPosition.origin.altitude;
				dbg_print("alt_error:");
				dbg_print_num(alt_error,10);
				dbg_print(" = -(");
				dbg_print_num(board->altitude_filtered,10);
				dbg_print(" + ");
				dbg_print_num(board->pressure.altitude_offset,10);
				dbg_print(") - ");
				dbg_print_num(board->imu1.attitude.localPosition.pos[2],10);
				dbg_print(" + ");
				dbg_print_num(board->imu1.attitude.localPosition.origin.altitude,10);
				dbg_print("\n");
			}
			tinterBaro = (get_micros()-board->pressure.last_update)/1000.0;
			board->imu1.attitude.localPosition.pos[2] += kp_alt/((float)(tinterBaro/2.5 + 1.0)) * alt_error;
			
			//vel_error[2]=board->pressure.vario_vz-board->imu1.attitude.vel[2];
			//board->imu1.attitude.vel[2] += kp_alt_v * vel_error[2];
	
			// correct velocity and accelerometer biases
			bias_correction.v[2]=alt_error;
			bias_correction=quat_global_to_local(board->imu1.attitude.qe, bias_correction);
			for (i=0; i<3; i++) 
			{
				board->imu1.attitude.vel_bf[i] += kp_alt_v/((float)(tinterBaro/2.5 + 1.0))*bias_correction.v[i];
				board->imu1.attitude.be[ACC_OFFSET+i] -= 0.005*kp_alt_v/((float)(tinterBaro/2.5 + 1.0))*bias_correction.v[i];
			}
						
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
			}
			tinterGps = get_millis() - board->GPS_data.timeLastMsg;
			if (tinterGps <= 1000)
			{
				for (i=0;i<3;i++)
				{
					pos_error[i] = local_coordinates.pos[i] - board->imu1.attitude.localPosition.pos[i];
					
					board->imu1.attitude.localPosition.pos[i] += kp_pos[i]/((float)(tinterGps/2.5 + 1.0)) * pos_error[i];
					vel_correction.v[i] = pos_error[i];
				}
				vel_correction = quat_global_to_local(board->imu1.attitude.qe, vel_correction);
				
				for (i=0;i<3;i++)
				{
					board->imu1.attitude.vel_bf[i] += kp_vel[i]/((float)(tinterGps/2.5 + 1.0)) * vel_correction.v[i];
					board->imu1.attitude.be[i+ACC_OFFSET] -= 0.001 /((float)(tinterGps/2.5 + 1.0))  *  vel_correction.v[i];
				}
				
			}
		}else{
			init_pos_gps();
		}
	}
}