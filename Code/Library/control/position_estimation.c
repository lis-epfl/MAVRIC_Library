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

board_hardware_t *board;

global_position_t global_gps_position;
local_coordinates_t local_coordinates;

float kp_pos[3],kp_vel[3];

uint32_t timeLastGpsMsg;

void init_pos_integration()
{
	board = get_board_hardware();
	
	board->init_gps_position = false;
	
	timeLastGpsMsg = 0;
	
	kp_pos[0] = 0.1;
	kp_pos[1] = 0.1;
	kp_pos[2] = 0.1;
	
	kp_vel[0] = 0.05;
	kp_vel[1] = 0.05;
	kp_vel[2] = 0.05;
	
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
	float pos_error[3];
	uint32_t tinter;
	int i;
	
	if (!(board->init_gps_position))
	{
		init_pos_gps();
	}
	
	if (board->init_gps_position && (board->simulation_mode == 0))
	{
		if (newValidGpsMsg(&timeLastGpsMsg))
		{
			//dbg_print("New valid message\n");
			global_gps_position.longitude = board->GPS_data.longitude;
			global_gps_position.latitude = board->GPS_data.latitude;
			global_gps_position.altitude = board->GPS_data.altitude;
			
			local_coordinates = global_to_local_position(global_gps_position,board->imu1.attitude.localPosition.origin);
		}
		tinter = get_millis() - board->GPS_data.timeLastMsg;
		if (tinter <= 1000)
		{
			for (i=0;i<3;i++)
			{
				pos_error[i] = local_coordinates.pos[i] - board->imu1.attitude.localPosition.pos[i];
				board->imu1.attitude.localPosition.pos[i] += kp_pos[i]/((float)(tinter/2.5 + 1.0)) * pos_error[i];
				board->imu1.attitude.vel[i] += kp_vel[i]/((float)(tinter/2.5 + 1.0)) * pos_error[i];
				//board->imu1.attitude.be[i+ACC_OFFSET] += 0.001 /((float)(tinter/2.5 + 1.0))  * pos_error[i]; FIX: convert to bf
			}
		}
	}
}