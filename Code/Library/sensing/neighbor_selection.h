/*
 * neighbor_selection.h
 *
 *  Created: 30.10.2013 09:40:13
 *  Author: ndousse
 */ 


#ifndef NEIGHBOR_SEL_H__
#define NEIGHBOR_SEL_H__

#include "mavlink_stream.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NUM_NEIGHBORS 15

#define SIZE_VHC_ORCA 5.0

#define ORCA_TIME_STEP_MILLIS 10.0

//2 seconds timeout limit
#define NEIGHBOR_TIMEOUT_LIMIT_MS 2000

typedef struct  {
	uint8_t neighborID;
	float position[3];
	float velocity[3];
	float size;
	uint32_t time_msg_received;
	float extrapolatedPosition[3];
}track_neighbor_t;

void init_neighbors(void);

void read_msg_from_neighbors(Mavlink_Received_t* rec);

void extrapolate_or_delete_position(track_neighbor_t listNeighbors[], uint8_t* number_of_neighbors);

#ifdef __cplusplus
}
#endif

#endif // NEIGHBOR_SEL_H__