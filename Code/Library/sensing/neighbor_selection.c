/*
 * neighbor_selection.c
 *
 *  Created: 30.10.2013 09:38:28
 *  Author: ndousse
 */ 


#include "neighbor_selection.h"
#include "central_data.h"
#include "coord_conventions.h"
#include "conf_platform.h"
#include <stdbool.h>

central_data_t *centralData;

void init_neighbors()
{
	centralData = get_central_data();
	centralData->number_of_neighbors = 0;
}


void read_msg_from_neighbors(Mavlink_Received_t* rec)
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
		
		globalPosNeighbor.longitude = (double)packet.lon / 10000000.0;
		globalPosNeighbor.latitude = (double)packet.lat / 10000000.0;
		globalPosNeighbor.altitude = (float)packet.alt / 1000.0;
		globalPosNeighbor.heading = (float)packet.hdg;
		
		localPosNeighbor = global_to_local_position(globalPosNeighbor,centralData->position_estimator.localPosition.origin);
		
		bool ID_found = false;
		i = 0;
		while ((!ID_found)&&(i<centralData->number_of_neighbors))
		{
			if (rec->msg.sysid == centralData->listNeighbors[i].neighborID)
			{
				ID_found = true;
			}else{
				i++;
			}
			
			//if (i>centralData->number_of_neighbors)
			//{
				//ID_found = true;
			//}
		}
		
		if (i>=centralData->number_of_neighbors)
		{
			if (centralData->number_of_neighbors < MAX_NUM_NEIGHBORS)
			{
				actualNeighbor = centralData->number_of_neighbors;
				centralData->number_of_neighbors++;
			}else{
				// This case shouldn't happen
				dbg_print("Error! There is more neighbors than planned!\n");
				actualNeighbor = centralData->number_of_neighbors-1;
			}
		}else{
			actualNeighbor = i;
		}
		
		
		
		centralData->listNeighbors[actualNeighbor].neighborID = rec->msg.sysid;
		
		for (i=0;i<3;i++)
		{
			centralData->listNeighbors[actualNeighbor].position[i] = localPosNeighbor.pos[i];
		}
		centralData->listNeighbors[actualNeighbor].velocity[X] = packet.vx / 100.0;
		centralData->listNeighbors[actualNeighbor].velocity[Y] = packet.vy / 100.0;
		centralData->listNeighbors[actualNeighbor].velocity[Z] = packet.vz / 100.0;
		
		centralData->listNeighbors[actualNeighbor].size = SIZE_VHC_ORCA;
		
		
		//dbg_print("Neighbor with ID ");
		//dbg_print_num(centralData->listNeighbors[actualNeighbor].neighborID,10);
		//dbg_print(" at position ");
		//dbg_print_vector(centralData->listNeighbors[actualNeighbor].position,3);
		//dbg_print(" with velocity ");
		//dbg_print_vector(centralData->listNeighbors[actualNeighbor].velocity,3);
		//dbg_print(" with relative position ");
		//float rel_pos[3];
		//uint8_t i;
		//for (i=0;i<3;i++)
		//{
			//rel_pos[i] = centralData->listNeighbors[actualNeighbor].position[i] - centralData->position_estimator.localPosition.pos[i];
		//}
		//dbg_print_vector(rel_pos,3);
		//dbg_print("\n");
		
	}
}