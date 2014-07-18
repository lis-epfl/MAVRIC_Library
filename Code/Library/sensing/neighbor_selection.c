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

void neighbors_selection_init(neighbor_t *neighborData, position_estimator_t *positionData, mavlink_message_handler_t *message_handler)
{
	neighborData->number_of_neighbors = 0;
	neighborData->positionData = positionData;
	
	// Add callbacks for onboard parameters requests
	mavlink_message_handler_msg_callback_t callback;

	callback.message_id 	= MAVLINK_MSG_ID_GLOBAL_POSITION_INT; // 39
	callback.sysid_filter 	= MAV_SYS_ID_ALL;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&neighbors_selection_read_message_from_neighbors;
	callback.module_struct 	= (handling_module_struct_t)		neighborData;
	mavlink_message_handler_add_msg_callback( message_handler, &callback );
	
		
	print_util_dbg_print("Neighbor selection initialized.\n");
}

void neighbors_selection_read_message_from_neighbors(neighbor_t *neighborData, mavlink_received_t* rec)
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
	
	if (rec->msg.sysid != mavlink_system.sysid)
	{
		global_position_t globalPosNeighbor;
		local_coordinates_t localPosNeighbor;
		uint8_t actualNeighbor;
		
		globalPosNeighbor.longitude = (double)packet.lon / 10000000.0f;
		globalPosNeighbor.latitude = (double)packet.lat / 10000000.0f;
		globalPosNeighbor.altitude = (float)packet.alt / 1000.0f;
		globalPosNeighbor.heading = (float)packet.hdg;
		
		localPosNeighbor = coord_conventions_global_to_local_position(globalPosNeighbor,neighborData->positionData->localPosition.origin);
		
		bool ID_found = false;
		i = 0;
		while ((!ID_found)&&(i < neighborData->number_of_neighbors))
		{
			if (rec->msg.sysid == neighborData->listNeighbors[i].neighborID)
			{
				ID_found = true;
			}
			else
			{
				i++;
			}
		}
		
		if (i >= neighborData->number_of_neighbors)
		{
			if (neighborData->number_of_neighbors < MAX_NUM_NEIGHBORS)
			{
				actualNeighbor = neighborData->number_of_neighbors;
				neighborData->number_of_neighbors++;
			}
			else
			{
				// This case shouldn't happen
				print_util_dbg_print("Error! There is more neighbors than planned!\n");
				actualNeighbor = neighborData->number_of_neighbors - 1;
			}
		}
		else
		{
			actualNeighbor = i;
		}
		
		
		
		neighborData->listNeighbors[actualNeighbor].neighborID = rec->msg.sysid;
		
		for (i = 0; i < 3; i++)
		{
			neighborData->listNeighbors[actualNeighbor].position[i] = localPosNeighbor.pos[i];
		}
		neighborData->listNeighbors[actualNeighbor].velocity[X] = packet.vx / 100.0f;
		neighborData->listNeighbors[actualNeighbor].velocity[Y] = packet.vy / 100.0f;
		neighborData->listNeighbors[actualNeighbor].velocity[Z] = packet.vz / 100.0f;
		
		neighborData->listNeighbors[actualNeighbor].size = SIZE_VHC_ORCA;
		
		neighborData->listNeighbors[actualNeighbor].time_msg_received = time_keeper_get_millis();
		
		//print_util_dbg_print("Neighbor with ID ");
		//print_util_dbg_print_num(neighborData->listNeighbors[actualNeighbor].neighborID,10);
		//print_util_dbg_print(" at position ");
		//print_util_dbg_print_vector(neighborData->listNeighbors[actualNeighbor].position,3);
		//print_util_dbg_print(" with velocity ");
		//print_util_dbg_print_vector(neighborData->listNeighbors[actualNeighbor].velocity,3);
		//print_util_dbg_print(" with relative position ");
		//float rel_pos[3];
		//uint8_t i;
		//for (i = 0; i < 3; i++)
		//{
			//rel_pos[i] = neighborData->listNeighbors[actualNeighbor].position[i] - neighborData->position_estimator.localPosition.pos[i];
		//}
		//print_util_dbg_print_vector(rel_pos,3);
		//print_util_dbg_print("\n");
		
	}
}

void neighbors_selection_extrapolate_or_delete_position(neighbor_t *neighborData)
{
	int32_t i, ind, indSup;
	uint32_t delta_t;
	
	uint32_t actualTime = time_keeper_get_millis();
	
	for (ind = 0; ind < neighborData->number_of_neighbors; ind++)
	{
		delta_t = actualTime- neighborData->listNeighbors[ind].time_msg_received;

		if (delta_t >= NEIGHBOR_TIMEOUT_LIMIT_MS)
		{
			// suppressing element ind
			for (indSup = ind; indSup < (neighborData->number_of_neighbors - 1); indSup++)
			{
				neighborData->listNeighbors[indSup] = neighborData->listNeighbors[indSup + 1];
			}
			(neighborData->number_of_neighbors)--;
			
		}
		else if (delta_t > ORCA_TIME_STEP_MILLIS)
		{
			// extrapolating the last known position assuming a constant velocity
			for(i = 0; i < 3; i++)
			{
				neighborData->listNeighbors[ind].extrapolatedPosition[i] = neighborData->listNeighbors[ind].position[i] + neighborData->listNeighbors[ind].velocity[i] *((float)delta_t);
			}
		}
		else
		{
			// taking the latest known position
			for (i = 0; i < 3; i++)
			{
				neighborData->listNeighbors[ind].extrapolatedPosition[i] = neighborData->listNeighbors[ind].position[i];
			}
		}
	}
}