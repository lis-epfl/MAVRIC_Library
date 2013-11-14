/*
 * neighbor_selection.h
 *
 *  Created: 30.10.2013 09:40:13
 *  Author: ndousse
 */ 


#ifndef NEIGHBOR_SEL_H__
#define NEIGHBOR_SEL_H__

#include "mavlink_stream.h"

#define MAX_NUM_NEIGHBORS 15

#define SIZE_VHC_ORCA 5.0

typedef struct  {
	uint8_t neighborID;
	float position[3];
	float velocity[3];
	float size;
}track_neighbor_t;

void init_neighbors();

void read_msg_from_neighbors(Mavlink_Received_t* rec);

#endif // NEIGHBOR_SEL_H__