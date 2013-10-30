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
	
	global_position_t globalPosNeighbor;
	local_coordinates_t localPosNeighbor;
	uint8_t actualNeighbor;
	
	globalPosNeighbor.longitude = packet.lon;
	globalPosNeighbor.latitude = packet.lat;
	globalPosNeighbor.altitude = packet.alt;
	globalPosNeighbor.heading = packet.hdg;
	
	localPosNeighbor = global_to_local_position(globalPosNeighbor,centralData->position_estimator.localPosition.origin);
	
	bool ID_found = true;
	i = 1;
	while (!ID_found)
	{
		if (rec->msg.sysid == centralData->listNeighbors[i].neighborID)
		{
			ID_found = false;
		}else{
			i++;
		}
		
		if (i>centralData->number_of_neighbors)
		{
			ID_found = false;
		}
	}
	
	if (i>centralData->number_of_neighbors)
	{
		if (centralData->number_of_neighbors < MAX_NUM_NEIGHBORS)
		{
			centralData->number_of_neighbors++;
			actualNeighbor = centralData->number_of_neighbors;
		}else{
			// This case shouldn't happen
			dbg_print("There is more neighbors than planned!\n");
			actualNeighbor = centralData->number_of_neighbors;
		}
	}else{
		actualNeighbor = i;
	}
	
	
	
	centralData->listNeighbors[actualNeighbor].neighborID = rec->msg.sysid;
	
	for (i=0;i<3;i++)
	{
		centralData->listNeighbors[actualNeighbor].position[i] = localPosNeighbor.pos[i];
	}
	centralData->listNeighbors[actualNeighbor].velocity[X] = packet.vx;
	centralData->listNeighbors[actualNeighbor].velocity[Y] = packet.vy;
	centralData->listNeighbors[actualNeighbor].velocity[Z] = packet.vz;
	
	centralData->listNeighbors[actualNeighbor].size = 1.0;
	
}