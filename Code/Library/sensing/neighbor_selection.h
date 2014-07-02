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
 * \file neighbor_selection.h
 *
 * This file decodes the message from the neighbors and computes the relative position and velocity in local coordinates
 */


#ifndef NEIGHBOR_SEL_H__
#define NEIGHBOR_SEL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_stream.h"

#define MAX_NUM_NEIGHBORS 15

#define SIZE_VHC_ORCA 5.0

#define ORCA_TIME_STEP_MILLIS 10.0

//2 seconds timeout limit
#define NEIGHBOR_TIMEOUT_LIMIT_MS 2000

/**
 * \brief The neighbor structure
 */
typedef struct  {
	uint8_t neighborID;					///< The mavlink ID of the vehicle
	float position[3];					///< The 3D position of the neighbor in m
	float velocity[3];					///< The 3D velocity of the neighbor in m/s
	float size;							///< The physical size of the neighbor in m
	uint32_t time_msg_received;			///< The time at which the message was received in ms
	float extrapolatedPosition[3];		///< The 3D position of the neighbor
}track_neighbor_t;						///< The structure of information about a neighbor 

/**
 * \brief	Initialize the neighbor selection module
 */
void init_neighbors(void);

/**
 * \brief	Decode the message and parse to the neighbor array
 *
 * \param	rec		the pointer to the mavlink message
 */
void read_msg_from_neighbors(Mavlink_Received_t* rec);

/**
 * \brief	Extrapolate the position of each UAS between two messages, deletes the message if time elapsed too long from last message
 *
 * \param	listNeighbors			the array of all neighbors
 * \param	number_of_neighbors		the pointer to the number of neighbors
 */
void extrapolate_or_delete_position(track_neighbor_t listNeighbors[], uint8_t* number_of_neighbors);

#ifdef __cplusplus
}
#endif

#endif // NEIGHBOR_SEL_H__