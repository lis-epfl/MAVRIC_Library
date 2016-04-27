/*******************************************************************************
 * Copyright (c) 2009-2015, MAV'RIC Development Team
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
 * \file neighbor_selection.cpp
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * 
 * \brief This file decodes the message from the neighbors and computes the relative position and velocity in local coordinates
 * 
 ******************************************************************************/


#include "communication/neighbor_selection.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/coord_conventions.h"
#include "util/print_util.h"
#include <stdbool.h>
#include "util/maths.h"
}

Neighbors::Neighbors(Position_estimation& position_estimation, State& state, const Mavlink_stream& mavlink_stream):
	position_estimation_(position_estimation),
	state_(state),
	mavlink_stream_(mavlink_stream)
{
	number_of_neighbors_ = 0;
	
	
	mean_comm_frequency_ = 0.0f;
	variance_comm_frequency_ = 0.0f;
	
	previous_time_ = time_keeper_get_ms();
	
	update_time_interval_ = 1000.0f; // 1 sec
	
	safe_size_ = safe_size__VHC;
	min_dist_ = 20.0f * safe_size__VHC + 1.0f;
	max_dist_ = -1.0f;
	
	uint8_t i;
	collision_log_.count_near_miss = 0;
	collision_log_.count_collision = 0;
	for (i = 0; i < MAX_NUM_NEIGHBORS; i++)
	{
		collision_log_.near_miss_flag[i] = false;
		collision_log_.collision_flag[i] = false;
		collision_log_.transition_flag[i] = false;
	}
	collision_dist_sqr_ = SQR(2.0f * SIZE_VHC);
	near_miss_dist_sqr_ = SQR(2.0f * safe_size__VHC);
}

void neighbor_selection_read_message_from_neighbors(Neighbors* neighbors, uint32_t sysid, mavlink_message_t* msg)
{
	
	 
	//Use this block for message debugging
	/*
	print_util_dbg_print("\n Received message with ID");
	print_util_dbg_print_num(msg->msgid, 10);
	print_util_dbg_print(" from system");
	print_util_dbg_print_num(msg->sysid, 10);
	print_util_dbg_print(" for component");
	print_util_dbg_print_num(msg->compid,10);
	print_util_dbg_print( "\r\n");
	*/
	
	
	uint8_t i;
	
	mavlink_global_position_int_t packet;

	mavlink_msg_global_position_int_decode(msg,&packet);
	//Check if coming from a neighbor
	
	if (msg->sysid != (uint8_t)sysid)
	{
		global_position_t global_pos_neighbor;
		local_position_t local_pos_neighbor;
		uint8_t actual_neighbor;
		
		global_pos_neighbor.longitude = (double)packet.lon / 10000000.0f;
		global_pos_neighbor.latitude = (double)packet.lat / 10000000.0f;
		global_pos_neighbor.altitude = (float)packet.alt / 1000.0f;
		global_pos_neighbor.heading = (float)packet.hdg;
		
		local_pos_neighbor = coord_conventions_global_to_local_position(global_pos_neighbor,neighbors->position_estimation_.local_position.origin);
		
		local_pos_neighbor.pos[2] = -packet.relative_alt / 1000.0f;
		
		bool ID_found = false;
		i = 0;
		while ((!ID_found)&&(i < neighbors->number_of_neighbors_))
		{
			if (msg->sysid == neighbors->neighbors_list_[i].neighbor_ID)
			{
				ID_found = true;
			}
			else
			{
				i++;
			}
		}
		
		if (i >= neighbors->number_of_neighbors_)
		{
			if (neighbors->number_of_neighbors_ < MAX_NUM_NEIGHBORS)
			{
				actual_neighbor = neighbors->number_of_neighbors_;
				neighbors->number_of_neighbors_++;
				neighbors->neighbors_list_[actual_neighbor].comm_frequency = 0.0f;
				neighbors->neighbors_list_[actual_neighbor].msg_count = 0;
				
				neighbors->comm_frequency_list_[(msg->sysid-1)%10] = 0.0f;
			}
			else
			{
				// This case shouldn't happen
				print_util_dbg_print("Error! There is more neighbors than planned!\n");
				actual_neighbor = neighbors->number_of_neighbors_ - 1;
			}
		}
		else
		{
			actual_neighbor = i;
		}

		neighbors->neighbors_list_[actual_neighbor].neighbor_ID = msg->sysid;
		
		for (i = 0; i < 3; i++)
		{
			neighbors->neighbors_list_[actual_neighbor].position[i] = local_pos_neighbor.pos[i];
		}
		neighbors->neighbors_list_[actual_neighbor].velocity[X] = packet.vx / 100.0f;
		neighbors->neighbors_list_[actual_neighbor].velocity[Y] = packet.vy / 100.0f;
		neighbors->neighbors_list_[actual_neighbor].velocity[Z] = packet.vz / 100.0f;
		
		neighbors->neighbors_list_[actual_neighbor].size = SIZE_VHC;
		
		neighbors->neighbors_list_[actual_neighbor].time_msg_received = time_keeper_get_ms();
		
		neighbors->neighbors_list_[actual_neighbor].msg_count++;
	}
}

bool neighbor_selection_telemetry_init(Neighbors* neighbors, Mavlink_message_handler* message_handler)
{
	bool init_success = true;
	
	// Add callbacks for onboard parameters requests
	Mavlink_message_handler::msg_callback_t callback;

	callback.message_id 	= MAVLINK_MSG_ID_GLOBAL_POSITION_INT; // 33
	callback.sysid_filter 	= MAV_SYS_ID_ALL;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (Mavlink_message_handler::msg_callback_func_t)	&neighbor_selection_read_message_from_neighbors;
	callback.module_struct 	= (Mavlink_message_handler::handling_module_struct_t)		neighbors;
	init_success &= message_handler->add_msg_callback(&callback);

	print_util_dbg_print("[NEIGHBOR SELECTION] Initialised.\r\n");
	
	return init_success;
}

void Neighbors::extrapolate_or_delete_position(void)
{
	int32_t i, ind, ind_sup;
	uint32_t delta_t;
	
	uint32_t actual_time = time_keeper_get_ms();
	
	for (ind = 0; ind < number_of_neighbors_; ind++)
	{
		delta_t = actual_time- neighbors_list_[ind].time_msg_received;

		if (delta_t >= NEIGHBOR_TIMEOUT_LIMIT_MS)
		{
			print_util_dbg_print("Suppressing neighbor number ");
			print_util_dbg_print_num(ind,10);
			print_util_dbg_print("\r\n");
			
			comm_frequency_list_[(neighbors_list_[ind].neighbor_ID-1)%10] = 0.0f;
			
			// suppressing element ind
			for (ind_sup = ind; ind_sup < (number_of_neighbors_ - 1); ind_sup++)
			{
				neighbors_list_[ind_sup] = neighbors_list_[ind_sup + 1];
			}
			(number_of_neighbors_)--;
			
			
		}
		else if (delta_t > ORCA_TIME_STEP_MILLIS)
		{
			// extrapolating the last known position assuming a constant velocity
			
			for(i = 0; i < 3; i++)
			{
				neighbors_list_[ind].extrapolated_position[i] = neighbors_list_[ind].position[i] + neighbors_list_[ind].velocity[i] *((float)delta_t/1000);
			}
		}
		else
		{
			// taking the latest known position
			for (i = 0; i < 3; i++)
			{
				neighbors_list_[ind].extrapolated_position[i] = neighbors_list_[ind].position[i];
			}
		}
	}
}

void Neighbors::compute_communication_frequency(void)
{
	uint16_t i;
	
	uint32_t actual_time = time_keeper_get_ms();
	
	if ((actual_time - previous_time_) >= update_time_interval_)
	{
		if (number_of_neighbors_ != 0)
		{
			//mavlink_message_t msg;
			
			mean_comm_frequency_ = 0.0f;
			variance_comm_frequency_ = 0.0f;
			
			for (i=0;i<number_of_neighbors_;++i)
			{
				neighbors_list_[i].comm_frequency = FREQ_LPF*neighbors_list_[i].comm_frequency + (1.0f-FREQ_LPF)*neighbors_list_[i].msg_count*1000.0f/(actual_time - previous_time_);
				comm_frequency_list_[(neighbors_list_[i].neighbor_ID-1)%10] = neighbors_list_[i].comm_frequency;
				
				neighbors_list_[i].msg_count = 0;
				mean_comm_frequency_ += neighbors_list_[i].comm_frequency;
			}

			mean_comm_frequency_ /= number_of_neighbors_;

			for (i=0;i<number_of_neighbors_;++i)
			{
				variance_comm_frequency_ += SQR(mean_comm_frequency_ - neighbors_list_[i].comm_frequency);
			}
			variance_comm_frequency_ /= number_of_neighbors_;
		}
		else
		{
			mean_comm_frequency_ = 0.0f;
			variance_comm_frequency_ = 0.0f;
		}

		comm_frequency_list_[(mavlink_stream_.sysid()-1)%10] = FREQ_LPF*comm_frequency_list_[(mavlink_stream_.sysid()-1)%10] + (1.0f-FREQ_LPF) * state_.msg_count *1000.0f/(actual_time- previous_time_);
		state_.msg_count = 0;
		
		previous_time_ = actual_time;
	}
}

void Neighbors::collision_log_smallest_distance(void)
{
	uint8_t ind, i;
	float relative_position[3];
	float distSqr, dist;
	
	min_dist_ = 20.0f * SIZE_VHC + 1.0f;
	max_dist_ = -1.0;

	if (state_.armed())
	{
		for (ind = 0; ind < number_of_neighbors_; ind++)
		{
			for (i = 0; i < 3; i++)
			{
				relative_position[i] = position_estimation_.local_position.pos[i] - neighbors_list_[ind].extrapolated_position[i];
			}
			distSqr = vectors_norm_sqr(relative_position);
		
			dist = sqrt(distSqr);
			min_dist_ = maths_f_min(min_dist_,dist);
			max_dist_ = maths_f_max(max_dist_,dist);

			if (distSqr < collision_dist_sqr_ && !collision_log_.collision_flag[ind])
			{
				collision_log_.count_collision++;
				if (collision_log_.count_near_miss != 0 && !collision_log_.transition_flag[ind])
				{
					collision_log_.count_near_miss--;
					collision_log_.transition_flag[ind] = true;
				}
				collision_log_.collision_flag[ind] = true;
				print_util_dbg_print("Collision with neighbor:");
				print_util_dbg_print_num(ind,10);
				print_util_dbg_print(", nb of collisions:");
				print_util_dbg_print_num(collision_log_.count_collision,10);
				print_util_dbg_print("\r\n");
			}
			else if (distSqr < near_miss_dist_sqr_ && !collision_log_.near_miss_flag[ind])
			{
				collision_log_.count_near_miss++;
				collision_log_.near_miss_flag[ind] = true;
				print_util_dbg_print("Near miss with neighbor:");
				print_util_dbg_print_num(ind,10);
				print_util_dbg_print(", nb of near miss:");
				print_util_dbg_print_num(collision_log_.count_near_miss,10);
				print_util_dbg_print("\r\n");
			}
			else if (distSqr > near_miss_dist_sqr_)
			{
				collision_log_.collision_flag[ind] = false;
				collision_log_.near_miss_flag[ind] = false;
				collision_log_.transition_flag[ind] = false;
			}
			else if (distSqr > collision_dist_sqr_)
			{
				collision_log_.collision_flag[ind] = false;
			}
		}
	}
	local_density_ = number_of_neighbors_ * 4.0f/3.0f*PI*max_dist_*max_dist_*max_dist_;
}
