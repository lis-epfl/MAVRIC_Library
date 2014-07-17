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
#include "position_estimation.h"
#include "mavlink_communication.h"

#define MAX_NUM_NEIGHBORS 15

#define SIZE_VHC_ORCA 5.0f

#define ORCA_TIME_STEP_MILLIS 10.0f

//2 seconds timeout limit
#define NEIGHBOR_TIMEOUT_LIMIT_MS 2000

/**
 * \brief The neighbor structure
 */
typedef struct
{
	uint8_t neighborID;					///< The mavlink ID of the vehicle
	float position[3];					///< The 3D position of the neighbor in m
	float velocity[3];					///< The 3D velocity of the neighbor in m/s
	float size;							///< The physical size of the neighbor in m
	uint32_t time_msg_received;			///< The time at which the message was received in ms
	float extrapolatedPosition[3];		///< The 3D position of the neighbor
}track_neighbor_t;						///< The structure of information about a neighbor 

typedef struct
{
		uint8_t number_of_neighbors;								///< The actual number of neighbors at a given time step
		float safe_size;											///< The safe size for collision avoidance
		track_neighbor_t listNeighbors[MAX_NUM_NEIGHBORS];
		position_estimator_t *positionData;
}neighbor_t;
/**
 * \brief	Initialize the neighbor selection module
 *
 * \param neighborData pointer to the neighbor struct
 * \param positionData pointer to the position estimator struct
 */
void neighbors_selection_init(neighbor_t *neighborData, position_estimator_t *positionData, mavlink_communication_t *mavlink_communication);

/**
 * \brief	Decode the message and parse to the neighbor array
 *
 * \param	rec		the pointer to the mavlink message
 */
void neighbors_selection_read_message_from_neighbors(neighbor_t *neighborData, mavlink_received_t* rec);

/**
 * \brief	Extrapolate the position of each UAS between two messages, deletes the message if time elapsed too long from last message
 *
 * \param	neighborData		the pointer to the neighbors struct
 */
void neighbors_selection_extrapolate_or_delete_position(neighbor_t *neighborData);

#ifdef __cplusplus
}
#endif

#endif // NEIGHBOR_SEL_H__