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
#include "central_data.h"
#include "coord_conventions.h"
#include "conf_platform.h"
#include "print_util.h"
#include <stdbool.h>

central_data_t *centralData;

void neighbors_selection_init()
{
	centralData = central_data_get_pointer_to_struct();
	centralData->number_of_neighbors = 0;
}

void neighbors_selection_read_message_from_neighbors(Mavlink_Received_t* rec)
{
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
		
		localPosNeighbor = coord_conventions_global_to_local_position(globalPosNeighbor,centralData->position_estimator.localPosition.origin);
		
		bool ID_found = false;
		i = 0;
		while ((!ID_found)&&(i<centralData->number_of_neighbors))
		{
			if (rec->msg.sysid == centralData->listNeighbors[i].neighborID)
			{
				ID_found = true;
			}
			else
			{
				i++;
			}
		}
		
		if (i>=centralData->number_of_neighbors)
		{
			if (centralData->number_of_neighbors < MAX_NUM_NEIGHBORS)
			{
				actualNeighbor = centralData->number_of_neighbors;
				centralData->number_of_neighbors++;
			}
			else
			{
				// This case shouldn't happen
				print_util_dbg_print("Error! There is more neighbors than planned!\n");
				actualNeighbor = centralData->number_of_neighbors-1;
			}
		}
		else
		{
			actualNeighbor = i;
		}
		
		
		
		centralData->listNeighbors[actualNeighbor].neighborID = rec->msg.sysid;
		
		for (i=0; i<3; i++)
		{
			centralData->listNeighbors[actualNeighbor].position[i] = localPosNeighbor.pos[i];
		}
		centralData->listNeighbors[actualNeighbor].velocity[X] = packet.vx / 100.0f;
		centralData->listNeighbors[actualNeighbor].velocity[Y] = packet.vy / 100.0f;
		centralData->listNeighbors[actualNeighbor].velocity[Z] = packet.vz / 100.0f;
		
		centralData->listNeighbors[actualNeighbor].size = SIZE_VHC_ORCA;
		
		centralData->listNeighbors[actualNeighbor].time_msg_received = time_keeper_get_millis();
		
		//print_util_dbg_print("Neighbor with ID ");
		//print_util_dbg_print_num(centralData->listNeighbors[actualNeighbor].neighborID,10);
		//print_util_dbg_print(" at position ");
		//print_util_dbg_print_vector(centralData->listNeighbors[actualNeighbor].position,3);
		//print_util_dbg_print(" with velocity ");
		//print_util_dbg_print_vector(centralData->listNeighbors[actualNeighbor].velocity,3);
		//print_util_dbg_print(" with relative position ");
		//float rel_pos[3];
		//uint8_t i;
		//for (i=0; i<3; i++)
		//{
			//rel_pos[i] = centralData->listNeighbors[actualNeighbor].position[i] - centralData->position_estimator.localPosition.pos[i];
		//}
		//print_util_dbg_print_vector(rel_pos,3);
		//print_util_dbg_print("\n");
		
	}
}

void neighbors_selection_extrapolate_or_delete_position(track_neighbor_t listNeighbors[], uint8_t* number_of_neighbors)
{
	int i, ind, indSup;
	uint32_t delta_t;
	
	uint32_t actualTime = time_keeper_get_millis();
	
	for (ind=0;ind<*number_of_neighbors;ind++)
	{
		delta_t = actualTime- listNeighbors[ind].time_msg_received;

		if (delta_t >= NEIGHBOR_TIMEOUT_LIMIT_MS)
		{
			// suppressing element ind
			for (indSup=ind; indSup<(*number_of_neighbors-1); indSup++)
			{
				listNeighbors[indSup] = listNeighbors[indSup + 1];
			}
			(*number_of_neighbors)--;
			
		}
		else if (delta_t > ORCA_TIME_STEP_MILLIS)
		{
			// extrapolating the last known position assuming a constant velocity
			for(i=0; i<3; i++)
			{
				listNeighbors[ind].extrapolatedPosition[i] = listNeighbors[ind].position[i] + listNeighbors[ind].velocity[i] *((float)delta_t);
			}
		}
		else
		{
			// taking the latest known position
			for (i=0; i<3; i++)
			{
				listNeighbors[ind].extrapolatedPosition[i] = listNeighbors[ind].position[i];
			}
		}
	}
}