/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file acoustic_recieve.c
 * 
 * \author MAV'RIC Team
 * \author Meysam Basiri
 *   
 * \brief Acoustic receive
 *
 ******************************************************************************/
 
 
#define KP_YAW 0.2

#include "acoustic_recieve.h"

#include "central_data.h"
#include "print_util.h"
#include "quick_trig.h"

central_data_t *centralData;

float Az[STORE_SIZE]={0,PI/2,-PI/4,0};
float El[STORE_SIZE]={0,0,-PI/4,0}; 
float position_previous[3];
float position_current[3]; 
float switch_previous;

quat_t acoustic_attitude;
quat_t acoustic_attitude_previous ;

float target_vect[3];
uint16_t Counter;

uint8_t first_Run=0, first_Run1=0;
//int16_t acoustic_input[64];

/*
************************************************************************************
************************************************************************************
*/


void recieve_acoustic()
{
	uint8_t buffer[6];
	int16_t Azimuth,Elevation;
	
	centralData->audio_data.NewData=0;
	
	if (first_Run==0) 
	{
		first_Run=1;
		buffer_clear(&(centralData->audio_buffer));	
	}
	
	//if(buffer_bytes_available(&(centralData->audio_buffer))>6) // To clear initialization delay ****maybe BETTER method required****
	//{
	//buffer_clear(&(centralData->audio_buffer));
	//}

	if(buffer_bytes_available(&(centralData->audio_buffer))>=6)
	{
		buffer[0]= buffer_get(&(centralData->audio_buffer));
		if (buffer[0]==254){
			buffer[1]= buffer_get(&(centralData->audio_buffer));
			buffer[2]= buffer_get(&(centralData->audio_buffer));
			buffer[3]= buffer_get(&(centralData->audio_buffer));
			buffer[4]= buffer_get(&(centralData->audio_buffer));
			buffer[5]= buffer_get(&(centralData->audio_buffer));
			if((-(buffer[1]+buffer[2]+buffer[3]+buffer[4])& 0xFF)==buffer[5]){ //checksum
				Azimuth=(buffer[1]<<8)|buffer[2];
				Elevation=(buffer[3]<<8)|buffer[4];
				centralData->audio_data.Azimuth=Azimuth;
				centralData->audio_data.Elevation=Elevation;
				centralData->audio_data.NewData=1;
				buffer_clear(&(centralData->audio_buffer));
			}
		}

	}
	
}


void process_acoustics(void)
{

	check_reliability();											// check the reliability of the current measurement
	
	if(centralData->audio_data.NewData==1 )
	{
		if (centralData->audio_data.ReliabeData==1 && centralData->acoustic_spin>0.7){
			set_waypoint_command_acoustic();							//calculate the approximate way point & follow azimuth
		}

		acoustic_attitude_previous=acoustic_attitude;
		position_previous[0]=position_current[0];
		position_previous[1]=position_current[1];
		position_previous[2]=position_current[2];
	}
	if (centralData->acoustic_spin<0.7 && switch_previous>=0.7)
	{
		wp_hold_init(centralData->position_estimator.localPosition);	//hold in position
	}
	
	//////////////for turning on the siren///////////////////////////
	if (centralData->acoustic_spin>0.9 && switch_previous<=0.9){ //siren was off , turn on
		//out_stream->put(out_stream->data, 254);
		//out_stream->put(out_stream->data, 103);
		// turn siren on
		turn_on_siren(centralData->telemetry_down_stream);
	}
	//if (centralData->acoustic_spin<0.9 && switch_previous>=0.9){ //siren was on , turn off
	//turn_on_siren(centralData->telemetry_down_stream);
	//// turn siren on
	//}
	/////////////////////////////////////////////////////////////////
	
	
	switch_previous=centralData->acoustic_spin;
}


void turn_on_siren(byte_stream_t *out_stream){
	out_stream->put(out_stream->data,254);
	out_stream->put(out_stream->data,131);
	out_stream->put(out_stream->data,44);
}
void turn_off_siren(byte_stream_t *out_stream){
	out_stream->put(out_stream->data,254);
	out_stream->put(out_stream->data,131);
	out_stream->put(out_stream->data,100);
}



void check_reliability(void)
{
	float  CosEl, SinEl, CosAz, SinAz, angulardiff=0;
	int8_t i;
	if (centralData->audio_data.NewData==0)
	{
		centralData->audio_data.ReliabeData=0;
		if (Counter<WAIT_LIMIT)
		Counter ++;
		else
		{
			Az[1]=0;
			Az[2]=PI/2;
			Az[3]=-PI/4;
			Counter=0;
			centralData->audio_data.Azimuth=-500;   // for plotting purposes
			centralData->audio_data.Elevation=-500;
		}
	}
	else
	{
		Counter=0;
		acoustic_attitude=centralData->imu1.attitude.qe;			    //store the current IMU attitude and position for later use
		position_current[0]=centralData->position_estimator.localPosition.pos[0];
		position_current[1]=centralData->position_estimator.localPosition.pos[1];
		position_current[2]=centralData->position_estimator.localPosition.pos[2];
		
		for (i=1; i<STORE_SIZE;i++)
		{
			Az[i-1]=Az[i];
			El[i-1]=El[i];
		}
		Az[STORE_SIZE-1]=centralData->audio_data.Azimuth*PI/180.0f;
		El[STORE_SIZE-1]=centralData->audio_data.Elevation*PI/180.0f;
		
		CosEl=quick_cos(El[STORE_SIZE-1]);
		SinEl=quick_sin(El[STORE_SIZE-1]);
		
		for (i=0; i<STORE_SIZE-1; i++)		//compute the total great arc circle between last bearing and the past 3 values
		{
			angulardiff+=quick_acos(CosEl*quick_cos(El[i])*quick_cos(Az[STORE_SIZE-1]-Az[i])+SinEl*quick_sin(El[i]));
		}
		
		if (angulardiff<RELIABILITY_ARC)
		{
			centralData->audio_data.ReliabeData=1;
			centralData->audio_data.ReliabeAz=Az[STORE_SIZE-1];
			centralData->audio_data.ReliabeEl=El[STORE_SIZE-1];
			//			counting=0;  //reset the counter
			CosAz=quick_cos(Az[STORE_SIZE-1]);
			SinAz=quick_sin(Az[STORE_SIZE-1]);
			target_vect[0]=CosAz*CosEl;
			target_vect[1]=SinAz*CosEl;
			target_vect[2]=SinEl;
		}
		else
		{
			centralData->audio_data.ReliabeData=0;
			//counting++;
		}
		
	}

}


void set_speed_command_acoustic(float rel_pos[], float dist2wpSqr)
{
	uint8_t i;
	float   norm_rel_dist, v_desiredxy,v_desiredz;
	//UQuat_t qtmp1, qtmp2;
	
	float dir_desired_bf[3], dir_desired[3], new_velocity[3];
	
	//float rel_heading;
	
	norm_rel_dist =  rel_pos[2];
	
	dir_desired_bf[2] = rel_pos[2];

	v_desiredz = f_min(centralData->cruise_speed,( centralData->dist2vel_gain * soft_zone(norm_rel_dist,centralData->softZoneSize)));
	
	if(centralData->audio_data.ReliabeEl<1.35){
		v_desiredxy = f_min(centralData->cruise_speed,(center_window_2(4.0f*centralData->audio_data.ReliabeAz) * (-9.0f*centralData->audio_data.ReliabeEl+(1.35*9))));
		}else{
		v_desiredxy=0;
	}
	
	if (v_desiredz *  f_abs(dir_desired_bf[Z]) > centralData->max_climb_rate * norm_rel_dist ) {
		v_desiredz = centralData->max_climb_rate * norm_rel_dist /f_abs(dir_desired_bf[Z]);
	}
	
	dir_desired_bf[X] = v_desiredxy * quick_cos(centralData->audio_data.ReliabeAz);
	dir_desired_bf[Y] = v_desiredxy * quick_sin(centralData->audio_data.ReliabeAz);
	dir_desired_bf[Z] = v_desiredz * dir_desired_bf[Z] / norm_rel_dist;
	
	
	for (i=0;i<3;i++)
	{
		new_velocity[i] = dir_desired_bf[i];
	}

	centralData->controls_nav.tvel[X] = new_velocity[X];
	centralData->controls_nav.tvel[Y] = new_velocity[Y];
	centralData->controls_nav.tvel[Z] = new_velocity[Z];
	centralData->controls_nav.rpy[YAW] = KP_YAW * centralData->audio_data.ReliabeAz;
}



void set_waypoint_command_acoustic()
{
	UQuat_t qtmp1, qtmp2;
	float target_vect_global[3], r ;
	qtmp1 = quat_from_vector(target_vect);
	qtmp2 = quat_local_to_global(acoustic_attitude_previous,qtmp1);
	target_vect_global[0] = qtmp2.v[0]; target_vect_global[1] = qtmp2.v[1]; target_vect_global[2] = qtmp2.v[2];  //unit vector to target
	//r=f_abs(position_previous[2]/target_vect_global[2]);			//distance to target on ground (computed from the hight of robot)
	//r=f_abs(1.0-target_vect_global[2])*15.0
	
	r=f_abs(quick_acos(target_vect_global[2]))*20;
	if (r>20.0)
	{
		r=20.0f;
	}
	if (r<3.0)
	{
		r=0.0f;
	}
	
	centralData->waypoint_hold_coordinates.pos[0]=r*target_vect_global[0]+position_previous[0];
	centralData->waypoint_hold_coordinates.pos[1]=r*target_vect_global[1]+position_previous[1];
	//centralData->controls_nav.theading=centralData->audio_data.Azimuth*PI/180;
	if (r>3.0)
	{
		centralData->waypoint_hold_coordinates.heading=atan2(target_vect_global[Y],target_vect_global[X]);
	}

	//centralData->waypoint_hold_coordinates.pos=
	//mavlink_system.sysid=33;
	// send integrated position (for now there is no GPS error correction...!!!)
	//global_position_t gpos=local_to_global_position(centralData->waypoint_hold_coordinates);
	//global_position_t gpos=local_to_global_position(centralData->position_estimator.localPosition);
	//mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
	//mavlink_msg_global_position_int_send(MAVLINK_COMM_0, get_millis(), gpos.latitude*10000000, gpos.longitude*10000000, gpos.altitude*1000.0, 1, centralData->position_estimator.vel[0]*100.0, centralData->position_estimator.vel[1]*100.0, centralData->position_estimator.vel[2]*100.0, centralData->imu1.attitude.om[2]);
	//if (first_Run1==0);
	//mavlink_system.sysid=MAVLINK_SYS_ID;
	
}