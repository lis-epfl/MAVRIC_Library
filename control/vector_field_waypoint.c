/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file vector_field_waypoint.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Vector field navigation using repulsors and attractors set through GPS waypoints
 *
 ******************************************************************************/


#include "vector_field_waypoint.h"
#include "coord_conventions.h"
#include "stdint.h"
#include "constants.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief 	Converts GPS Waypoint coordinates to local NED frame 
 * 
 * \param 	waypoint 	Input waypoint
 * \param 	origin 		Origin of the local NED frame
 * 
 * \return  			waypoint in NED coordinates
 */
static waypoint_struct_t convert_waypoint_to_local_ned(const waypoint_struct_t* waypoint, const global_position_t* origin);


/**
 * \brief 		Vector field for floor avoidance 
 * 
 * \details 	Computes a 3D velocity vector keeping the MAV from colliding with the floor
 * 				 
 * \param 		pos 		Current position of the MAV (input)
 * \param  		altitude	Threshold altitude (>0) under which the vector field is active (input)
 * \param 		vector		Velocity command vector (output)
 */
static void vector_field_floor(const float pos[3], const float altitude, float vector[3]);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static waypoint_struct_t convert_waypoint_to_local_ned(const waypoint_struct_t* waypoint_in, const global_position_t* origin)
{
	global_position_t waypoint_global;
	local_coordinates_t waypoint_local;

	// Init new waypoint
	waypoint_struct_t waypoint = *waypoint_in;
	
	switch( waypoint_in->frame )
	{
		case MAV_FRAME_GLOBAL:					/* Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */
			
			waypoint_global.latitude 	= waypoint_in->x;
			waypoint_global.longitude 	= waypoint_in->y;
			waypoint_global.altitude 	= waypoint_in->z;
			waypoint_local				= coord_conventions_global_to_local_position(waypoint_global, *origin);
			
			waypoint.frame 	= MAV_FRAME_LOCAL_NED;
			waypoint.x 		= waypoint_local.pos[X];
			waypoint.y 		= waypoint_local.pos[Y];
			waypoint.z 		= waypoint_local.pos[Z];
		break;

		case MAV_FRAME_LOCAL_NED:				/* Local coordinate frame, Z-up (x: north, y: east, z: down). | */
		case MAV_FRAME_MISSION:					/* NOT a coordinate frame, indicates a mission command. | */
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:		/* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */
		case MAV_FRAME_LOCAL_ENU:				/* Local coordinate frame, Z-down (x: east, y: north, z: up) | */
		case MAV_FRAME_LOCAL_OFFSET_NED:		/* Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | */
		case MAV_FRAME_BODY_NED:				/* Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | */
		case MAV_FRAME_BODY_OFFSET_NED:			/* Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | */
		case MAV_FRAME_GLOBAL_TERRAIN_ALT:		/* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
		case MAV_FRAME_ENUM_END:
			// do not use this waypoint				
			waypoint.command = 0;
		break;
	}

	return waypoint;
}


static void vector_field_floor(const float pos[3], const float altitude, float vector[3])
{
	// The horizontal and vertical components are always null
	vector[X] = 0.0f;
	vector[Y] = 0.0f;
	
	if( pos[2] >= 0.0f ) 			
	{
		// Avoid division per 0
		vector[Z] = 0.0f;
	}
	else if( pos[2] < -altitude )	
	{
		// High altitude
		vector[Z] = 0.0f;
	}
	else
	{
		// Low altitude
		vector[Z] = (1.0f / pos[2]) * (altitude - pos[2]);
	}
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void vector_field_waypoint_init(vector_field_waypoint_t* vector_field, const vector_field_waypoint_conf_t* config, const mavlink_waypoint_handler_t* waypoint_handler, const position_estimation_t* pos_est, velocity_command_t* velocity_command)
{
	// Init dependencies
	vector_field->waypoint_handler 	= waypoint_handler;
	vector_field->pos_est 			= pos_est;
	vector_field->velocity_command 	= velocity_command;
}

void vector_field_waypoint_update(vector_field_waypoint_t* vector_field)
{
	float tmp_vector[3];

	// Re-init velocity command
	vector_field->velocity_command->mode 	= VELOCITY_COMMAND_MODE_GLOBAL;
	vector_field->velocity_command->xyz[X] 	= 0.0f;
	vector_field->velocity_command->xyz[Y] 	= 0.0f;
	vector_field->velocity_command->xyz[Z] 	= 0.0f;

	// Add floor vector field
	vector_field_floor(	vector_field->pos_est->local_position.pos, 
						20, 
						tmp_vector);
	vector_field->velocity_command->xyz[X] += tmp_vector[X];
	vector_field->velocity_command->xyz[Y] += tmp_vector[Y];
	vector_field->velocity_command->xyz[Z] += tmp_vector[Z];

	// Go through waypoint list
	for (uint16_t i = 0; i < vector_field->waypoint_handler->number_of_waypoints; ++i)
	{
		// Get waypoint in NED coordinates
		waypoint_struct_t waypoint = convert_waypoint_to_local_ned( &vector_field->waypoint_handler->waypoint_list[i],
																	&vector_field->pos_est->local_position.origin );

		switch( waypoint.command )
		{
			default:
				vector_field->velocity_command->xyz[X] += 0.2 * (waypoint.x - vector_field->pos_est->local_position.pos[X]);
				vector_field->velocity_command->xyz[Y] += 0.2 * (waypoint.y - vector_field->pos_est->local_position.pos[Y]);
				vector_field->velocity_command->xyz[Z] += 0.2 * (waypoint.z - vector_field->pos_est->local_position.pos[Z]);
			break;
		}
	};
}