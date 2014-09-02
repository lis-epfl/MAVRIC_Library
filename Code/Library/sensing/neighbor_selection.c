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
 * \file neighbor_selection.c
 *
 * This file decodes the message from the neighbors and computes the relative position and velocity in local coordinates
 */


#include "neighbor_selection.h"
#include "coord_conventions.h"
#include "conf_platform.h"
#include "print_util.h"
#include "time_keeper.h"
#include <stdbool.h>

void neighbors_selection_init(neighbors_t *neighbors, position_estimator_t *position_estimator, mavlink_message_handler_t *message_handler, const mavlink_stream_t* mavlink_stream)
{
	neighbors->number_of_neighbors = 0;
	neighbors->position_estimator = position_estimator;
	neighbors->mavlink_stream = mavlink_stream;
	
	// Add callbacks for onboard parameters requests
	mavlink_message_handler_msg_callback_t callback;

	callback.message_id 	= MAVLINK_MSG_ID_GLOBAL_POSITION_INT; // 39
	callback.sysid_filter 	= MAV_SYS_ID_ALL;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&neighbors_selection_read_message_from_neighbors;
	callback.module_struct 	= (handling_module_struct_t)		neighbors;
	mavlink_message_handler_add_msg_callback( message_handler, &callback );
	
		
	print_util_dbg_print("Neighbor selection initialized.\r\n");
}

void neighbors_selection_read_message_from_neighbors(neighbors_t *neighbors, mavlink_received_t* rec)
{
	
	 
	//Use this block for message debugging
	/*
	print_util_dbg_print("\n Received message with ID");
	print_util_dbg_print_num(rec->msg.msgid, 10);
	print_util_dbg_print(" from system");
	print_util_dbg_print_num(rec->msg.sysid, 10);
	print_util_dbg_print(" for component");
	print_util_dbg_print_num(rec->msg.compid,10);
	print_util_dbg_print( "\n");
	*/
	
	uint8_t i;
	
	mavlink_global_position_int_t packet;
	mavlink_msg_global_position_int_decode(&rec->msg,&packet);
	//Check if coming from a neighbor
	
	if (rec->msg.sysid != neighbors->mavlink_stream->sysid)
	{
		global_position_t global_pos_neighbor;
		local_coordinates_t local_pos_neighbor;
		uint8_t actual_neighbor;
		
		global_pos_neighbor.longitude = (double)packet.lon / 10000000.0f;
		global_pos_neighbor.latitude = (double)packet.lat / 10000000.0f;
		global_pos_neighbor.altitude = (float)packet.alt / 1000.0f;
		global_pos_neighbor.heading = (float)packet.hdg;
		
		local_pos_neighbor = coord_conventions_global_to_local_position(global_pos_neighbor,neighbors->position_estimator->local_position.origin);
		
		bool ID_found = false;
		i = 0;
		while ((!ID_found)&&(i < neighbors->number_of_neighbors))
		{
			if (rec->msg.sysid == neighbors->neighbors_list[i].neighbor_ID)
			{
				ID_found = true;
			}
			else
			{
				i++;
			}
		}
		
		if (i >= neighbors->number_of_neighbors)
		{
			if (neighbors->number_of_neighbors < MAX_NUM_NEIGHBORS)
			{
				actual_neighbor = neighbors->number_of_neighbors;
				neighbors->number_of_neighbors++;
			}
			else
			{
				// This case shouldn't happen
				print_util_dbg_print("Error! There is more neighbors than planned!\n");
				actual_neighbor = neighbors->number_of_neighbors - 1;
			}
		}
		else
		{
			actual_neighbor = i;
		}
		
		
		
		neighbors->neighbors_list[actual_neighbor].neighbor_ID = rec->msg.sysid;
		
		for (i = 0; i < 3; i++)
		{
			neighbors->neighbors_list[actual_neighbor].position[i] = local_pos_neighbor.pos[i];
		}
		neighbors->neighbors_list[actual_neighbor].velocity[X] = packet.vx / 100.0f;
		neighbors->neighbors_list[actual_neighbor].velocity[Y] = packet.vy / 100.0f;
		neighbors->neighbors_list[actual_neighbor].velocity[Z] = packet.vz / 100.0f;
		
		neighbors->neighbors_list[actual_neighbor].size = SIZE_VHC_ORCA;
		
		neighbors->neighbors_list[actual_neighbor].time_msg_received = time_keeper_get_millis();
		
		//print_util_dbg_print("Neighbor with ID ");
		//print_util_dbg_print_num(neighbors->neighbors_list[actual_neighbor].neighbor_ID,10);
		//print_util_dbg_print(" at position ");
		//print_util_dbg_print_vector(neighbors->neighbors_list[actual_neighbor].position,3);
		//print_util_dbg_print(" with velocity ");
		//print_util_dbg_print_vector(neighbors->neighbors_list[actual_neighbor].velocity,3);
		//print_util_dbg_print(" with relative position ");
		//float rel_pos[3];
		//uint8_t i;
		//for (i = 0; i < 3; i++)
		//{
			//rel_pos[i] = neighbors->neighbors_list[actual_neighbor].position[i] - neighbors->position_estimator.local_position.pos[i];
		//}
		//print_util_dbg_print_vector(rel_pos,3);
		//print_util_dbg_print("\n");
		
	}
}

void neighbors_selection_extrapolate_or_delete_position(neighbors_t *neighbors)
{
	int32_t i, ind, ind_sup;
	uint32_t delta_t;
	
	uint32_t actual_time = time_keeper_get_millis();
	
	for (ind = 0; ind < neighbors->number_of_neighbors; ind++)
	{
		delta_t = actual_time- neighbors->neighbors_list[ind].time_msg_received;

		if (delta_t >= NEIGHBOR_TIMEOUT_LIMIT_MS)
		{
			// suppressing element ind
			for (ind_sup = ind; ind_sup < (neighbors->number_of_neighbors - 1); ind_sup++)
			{
				neighbors->neighbors_list[ind_sup] = neighbors->neighbors_list[ind_sup + 1];
			}
			(neighbors->number_of_neighbors)--;
			
		}
		else if (delta_t > ORCA_TIME_STEP_MILLIS)
		{
			// extrapolating the last known position assuming a constant velocity
			for(i = 0; i < 3; i++)
			{
				neighbors->neighbors_list[ind].extrapolated_position[i] = neighbors->neighbors_list[ind].position[i] + neighbors->neighbors_list[ind].velocity[i] *((float)delta_t);
			}
		}
		else
		{
			// taking the latest known position
			for (i = 0; i < 3; i++)
			{
				neighbors->neighbors_list[ind].extrapolated_position[i] = neighbors->neighbors_list[ind].position[i];
			}
		}
	}
}