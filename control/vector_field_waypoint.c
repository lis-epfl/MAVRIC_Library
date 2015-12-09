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
#include "vectors.h"

#include "print_util.h"

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
 * \param 		pos_mav 	Current position of the MAV (input)
 * \param  		altitude	Threshold altitude (>0) under which the vector field is active (input)
 * \param 		vector		Velocity command vector (output)
 */
static void vector_field_floor(const float pos_mav[3], const float altitude, float vector[3]);


/**
 * \brief 		Vector field for attractor object
 * 
 * \details 	Computes a 3D velocity vector attracting the MAV towards a 3D location
 * 				 
 * \param 		rel_pos 		Current relative position of the MAV to the waypoint (input)
 * \param 		attractiveness	Weight given to this object
 * \param 		vector			Velocity command vector (output)
 */
static void vector_field_attractor(const float rel_pos[3], const float attractiveness, float vector[3]);


/**
 * \brief 		Vector field for repulsive cylinder
 * 
 * \details 	Computes a 3D velocity vector pushing the MAV away from a vertical cylinder
 * 				 
 * \param 		rel_pos 		Current relative position of the MAV to the waypoint (input)
 * \param 		repulsiveness	Weight given to this object (input)
 * \param 		max_range		Range of action (in m)
 * \param 		safety_radius	Minimum distance the object can be approached (in m)
 * \param 		vector			Velocity command vector (output)
 */
static void vector_field_repulsor_cylinder(const float rel_pos[3], const float repulsiveness, const float max_range, const float safety_radius, float vector[3]);


/**
 * \brief 		Vector field for repulsive object
 * 
 * \details 	Computes a 3D velocity vector pushing the MAV away from a 3D location
 * 				 
 * \param 		rel_pos 		Current relative position of the MAV to the waypoint (input)
 * \param 		repulsiveness	Weight given to this object
 * \param 		max_range		Range of action (in m)
 * \param 		safety_radius	Minimum distance the object can be approached (in m)
 * \param 		vector			Velocity command vector (output)
 */
static void vector_field_repulsor_sphere(const float rel_pos[3], const float repulsiveness, const float max_range, const float safety_radius, float vector[3]);

/**
 * \brief 		Change the reference frame of the waypoint
 * 				 
 * \param 		vector_field 	The pointer to the vector field structure
 * \param 		goal_pos 		The goal position in global frame (input)
 * \param 		new_goal_pos	The new goal position in the correct reference frame
 */
static void vector_field_reference_frame(vector_field_waypoint_t* vector_field, float goal_pos[3], float new_goal_pos[3]);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static waypoint_struct_t convert_waypoint_to_local_ned(const waypoint_struct_t* waypoint_in, const global_position_t* origin)
{
	global_position_t waypoint_global;
	local_coordinates_t waypoint_local;

	// Init new waypoint
	waypoint_struct_t waypoint = *waypoint_in;
	waypoint.command 	= waypoint_in->command;
	waypoint.param1 	= waypoint_in->param1;
	waypoint.param2 	= waypoint_in->param2;
	waypoint.param3 	= waypoint_in->param3;
	waypoint.param4 	= waypoint_in->param4;
	
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


/**
 * \brief 		Vector field for floor avoidance 
 * 
 * \details 	Computes a 3D velocity vector keeping the MAV from colliding with the floor
 * 				 
 * \param 		pos_mav 	Current position of the MAV (input)
 * \param  		altitude	Threshold altitude (>0) under which the vector field is active (input)
 * \param 		vector		Velocity command vector (output)
 */
static void vector_field_floor(const float pos_mav[3], const float altitude, float vector[3])
{
	// The horizontal and vertical components are always null
	vector[X] = 0.0f;
	vector[Y] = 0.0f;
	
	if( pos_mav[2] >= 0.0f ) 			
	{
		// Avoid division by 0
		vector[Z] = 0.0f;
	}
	else
	{
		/**
		 *  Student code Here
		 */
		
		if( pos_mav[2] < -altitude )	
		{
			// High altitude
			vector[Z] = 0.0f;
		}
		else
		{
			// Low altitude
			vector[Z] = (1.0f / pos_mav[2]) * (altitude - pos_mav[2]);
		}

		/**
		 *  End of Student code
		 */

	}
}


/**
 * \brief 		Vector field for attractor object
 * 
 * \details 	Computes a 3D velocity vector attracting the MAV towards a 3D location
 * 				 
 * \param 		rel_pos 		Current relative position of the MAV to the waypoint (input)
 * \param 		attractiveness	Weight given to this object (input)
 * \param 		vector			Velocity command vector (output)
 */
static void vector_field_attractor(const float rel_pos[3], const float attractiveness, float vector[3])
{
	float direction[3] = {0.0f, 0.0f, 0.0f};	// desired direction (unit vector)
	float speed = 0.0f;							// desired speed

	// Compute norm of this vector
	float dist_to_object = vectors_norm(rel_pos);

	// Get desired direction
	direction[X] = rel_pos[X] / dist_to_object;
	direction[Y] = rel_pos[Y] / dist_to_object;
	direction[Z] = rel_pos[Z] / dist_to_object;
	
	// Compute desired speed
	speed = attractiveness * dist_to_object;

	/**
	 *  End of Student code
	 */

	vector[X] = direction[X] * speed;
	vector[Y] = direction[Y] * speed;
	vector[Z] = direction[Z] * speed;
}


/**
 * \brief 		Vector field for repulsive cylinder
 * 
 * \details 	Computes a 3D velocity vector pushing the MAV away from a vertical cylinder
 * 				 
 * \param 		rel_pos 		Current relative position of the MAV to the waypoint (input)
 * \param 		repulsiveness	Weight given to this object (input)
 * \param 		max_range		Range of action (in m)
 * \param 		safety_radius	Minimum distance the object can be approached (in m)
 * \param 		vector			Velocity command vector (output)
 */
static void vector_field_repulsor_cylinder(const float rel_pos[3], const float repulsiveness, const float max_range, const float safety_radius, float vector[3])
{
	float direction[3] = {0.0f, 0.0f, 0.0f};	// desired direction (unit vector)
	float speed = 0.0f;							// desired speed

	/**
	 *  Student code Here
	 */

	// Compute vector from MAV to goal 
	float mav_to_obj[3] = { rel_pos[X],
							rel_pos[Y],
							0.0f };

	// Compute norm of this vector
	float dist_to_object = vectors_norm(mav_to_obj);

	// Get desired direction
	direction[X] = mav_to_obj[X] / dist_to_object;
	direction[Y] = mav_to_obj[Y] / dist_to_object;
	direction[Z] = mav_to_obj[Z] / dist_to_object;
	
	// Compute desired speed
	if(	dist_to_object <= safety_radius )
	{
		speed = 0.0f;
	}
	else if( dist_to_object > max_range )
	{
		speed = 0.0f;
	}
	else
	{
		speed = - repulsiveness  / (dist_to_object - safety_radius) * (max_range - dist_to_object);
	}

	/**
	 *  End of Student code
	 */

	vector[X] = direction[X] * speed;
	vector[Y] = direction[Y] * speed;
	vector[Z] = direction[Z] * speed;
}


/**
 * \brief 		Vector field for repulsive object
 * 
 * \details 	Computes a 3D velocity vector pushing the MAV away from a 3D location
 * 				 
 * \param 		rel_pos 		Current relative position of the MAV to the waypoint (input)
 * \param 		repulsiveness	Weight given to this object (input)
 * \param 		max_range		Range of action (in m)
 * \param 		safety_radius	Minimum distance the object can be approached (in m)
 * \param 		vector			Velocity command vector (output)
 */
static void vector_field_repulsor_sphere(const float rel_pos[3], const float repulsiveness, const float max_range, const float safety_radius, float vector[3])
{
	float direction[3] = {0.0f, 0.0f, 0.0f};	// desired direction (unit vector)
	float speed = 0.0f;							// desired speed

	// Compute norm of this vector
	float dist_to_object = vectors_norm(rel_pos);

	// Get desired direction
	direction[X] = rel_pos[X] / dist_to_object;
	direction[Y] = rel_pos[Y] / dist_to_object;
	direction[Z] = rel_pos[Z] / dist_to_object;
	
	// Compute desired speed
	if(	dist_to_object <= safety_radius )
	{
		speed = 0.0f;
	}
	else if( dist_to_object > max_range )
	{
		speed = 0.0f;
	}
	else
	{
		speed = - repulsiveness  / (dist_to_object - safety_radius) * (max_range - dist_to_object);
	}

	/**
	 *  End of Student code
	 */

	vector[X] = direction[X] * speed;
	vector[Y] = direction[Y] * speed;
	vector[Z] = direction[Z] * speed;
}

static void vector_field_reference_frame(vector_field_waypoint_t* vector_field, float goal_pos[3], float new_goal_pos[3])
{
	aero_attitude_t attitude_yaw;
	quat_t q_rot, vel_local;

	uint32_t i;

	switch(vector_field->command_mode)
	{
		case VELOCITY_COMMAND_MODE_GLOBAL:
			// Already in global
			for (i = 0; i < 3; ++i)
			{
				new_goal_pos[i] = goal_pos[i];
			}
			
			break;
			
		case VELOCITY_COMMAND_MODE_LOCAL:
			// Transform global to local
			vel_local = quaternions_global_to_local(vector_field->ahrs->qe, quaternions_create_from_vector(goal_pos));
			new_goal_pos[0] = vel_local.v[0];
			new_goal_pos[1] = vel_local.v[1];
			new_goal_pos[2] = vel_local.v[2];
			break;
			
		case VELOCITY_COMMAND_MODE_SEMI_LOCAL:
		default:
			// Transform global to semi-local
			attitude_yaw = coord_conventions_quat_to_aero(vector_field->ahrs->qe);
			attitude_yaw.rpy[0] = 0.0f;
			attitude_yaw.rpy[1] = 0.0f;
			attitude_yaw.rpy[2] = -attitude_yaw.rpy[2];
			q_rot = coord_conventions_quaternion_from_aero(attitude_yaw);

			quaternions_rotate_vector(q_rot, goal_pos, new_goal_pos);
			break;
	}
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void vector_field_waypoint_init(vector_field_waypoint_t* vector_field, const vector_field_waypoint_conf_t* config, const mavlink_waypoint_handler_t* waypoint_handler, const position_estimation_t* pos_est, velocity_command_t* velocity_command, ahrs_t* ahrs, control_command_t* controls_nav)
{
	// Init dependencies
	vector_field->waypoint_handler 	= waypoint_handler;
	vector_field->pos_est 			= pos_est;
	vector_field->velocity_command 	= velocity_command;
	vector_field->ahrs				= ahrs;

	vector_field->controls_nav = controls_nav;
	
	// Copy config
	vector_field->command_mode = config->command_mode;
	vector_field->floor_altitude = config->floor_altitude;
	vector_field->velocity_max = config->velocity_max;
}


task_return_t vector_field_waypoint_update(vector_field_waypoint_t* vector_field)
{
	float tmp_vector[3];
	float pos_obj[3];
	float rel_pos[3];
	float dir_desired[3];

	uint32_t j;

	// Re-init velocity command
	vector_field->velocity_command->mode 	= vector_field->command_mode;
	dir_desired[X] 	= 0.0f;
	dir_desired[Y] 	= 0.0f;
	dir_desired[Z] 	= 0.0f;

	// Compute vector field for floor avoidance
	vector_field_floor(	vector_field->pos_est->local_position.pos, 
						vector_field->floor_altitude, 
						dir_desired );

	// Go through waypoint list
	for (uint16_t i = 0; i < vector_field->waypoint_handler->number_of_waypoints; ++i)
	{
		// Get waypoint in NED coordinates
		waypoint_struct_t waypoint = convert_waypoint_to_local_ned( &vector_field->waypoint_handler->waypoint_list[i],
																	&vector_field->pos_est->local_position.origin );

		// Get object position
		pos_obj[X] = waypoint.x;
		pos_obj[Y] = waypoint.y;
		pos_obj[Z] = waypoint.z;

		for (j = 0; j < 3; ++j)
		{
			rel_pos[j] = vector_field->pos_est->local_position.pos[j] - pos_obj[j];
		}
		

		switch( waypoint.command )
		{
			// Attractive object
			case 22:
				vector_field_attractor(	rel_pos,
										waypoint.param1, 	// attractiveness
										tmp_vector);
			break;

			// Cylindrical repulsive object
			case 17:
				vector_field_repulsor_cylinder( rel_pos,
												waypoint.param1, 	// repulsiveness
												waypoint.param2, 	// max_range
												waypoint.param3, 	// safety_radius
												tmp_vector );
			break;

			// Spherical repulsive object
			case 18:
				vector_field_repulsor_sphere( 	rel_pos,
												waypoint.param1, 	// repulsiveness
												waypoint.param2,  	// max_range
												waypoint.param3, 	// safety_radius
												tmp_vector );
			break;

			// Circular waypoint
			case 16:
				vector_field_circular_waypoint(	rel_pos, 
												waypoint.param1, 	// attractiveness
												1.0f, 				// attractiveness2
												waypoint.param2, 	// cruise_speed
												waypoint.param3,	// radius
												tmp_vector );
			break;

			// Unknown object
			default:
				tmp_vector[X] = 0.0f;
				tmp_vector[Y] = 0.0f;
				tmp_vector[Z] = 0.0f;
			break;
		}

		// Add vector field to the velocity command
		dir_desired[X] += tmp_vector[X];
		dir_desired[Y] += tmp_vector[Y];
		dir_desired[Z] += tmp_vector[Z];
	}

	// Limit velocity
	float vel_norm = vectors_norm(dir_desired);
	if( vel_norm > vector_field->velocity_max )
	{
		dir_desired[X] = vector_field->velocity_max * dir_desired[X] / vel_norm;
		dir_desired[Y] = vector_field->velocity_max * dir_desired[Y] / vel_norm;
		dir_desired[Z] = vector_field->velocity_max * dir_desired[Z] / vel_norm;
	}
	
	// Transform the velocity command into the correct frame
	float dir_desired_frame[3];
	vector_field_reference_frame(vector_field, dir_desired, dir_desired_frame);

	
	vector_field->velocity_command->xyz[X] = dir_desired_frame[X];
	vector_field->velocity_command->xyz[Y] = dir_desired_frame[Y];
	vector_field->velocity_command->xyz[Z] = dir_desired_frame[Z];
	
	return TASK_RUN_SUCCESS;
}

bool vector_field_circular_waypoint(const float rel_pos[3], const float attractiveness, const float attractiveness2, const float cruise_speed, const float radius, float vector[3])
{
	bool speed_good = true;
	float direction_radial[3] 		= {0.0f, 0.0f, 0.0f};	// desired direction (unit vector) toward the closest point of the circle
	float direction_tangential[3] 	= {0.0f, 0.0f, 0.0f};	// desired direction (unit vector) tangential to the circle (circular motion)
	float speed_radial 				= 0.0f;					// desired radial speed
	float speed_tangential 			= 0.0f;					// desired tangential speed
	
	// Compute distances
	float dist_to_object = vectors_norm(rel_pos);
	float horiz_dist_to_object = sqrtf(rel_pos[X]*rel_pos[X] + rel_pos[Y]*rel_pos[Y]);
	float horiz_dist_to_circle = (horiz_dist_to_object >= maths_f_abs(radius)) ? (horiz_dist_to_object - maths_f_abs(radius)) : (maths_f_abs(radius) - horiz_dist_to_object);
	
	// Avoid division by 0 if we are on the center of the circle...
	if(dist_to_object >= 0.001f)
	{
		// Compute point C, the point on the circle which is the nearest from the current position A (defining current point A as (0,0,0))
		float AC[3] = {0.0f, 0.0f, 0.0f};
		AC[X] = horiz_dist_to_circle * rel_pos[X] / horiz_dist_to_object;
		AC[Y] = horiz_dist_to_circle * rel_pos[Y] / horiz_dist_to_object;
		AC[Z] = rel_pos[Z];
		if(horiz_dist_to_object < maths_f_abs(radius))
		{
			AC[X] = -AC[X];
			AC[Y] = -AC[Y];
		}
		
		// Compute radial direction
		float dist_to_circle = vectors_norm(AC);
		if(dist_to_circle >= 0.001f)					// Just to be 100% sure
		{
			direction_radial[X] = AC[X] / dist_to_circle;
			direction_radial[Y] = AC[Y] / dist_to_circle;
			#warning clip vertical velocity
			direction_radial[Z] = AC[Z] / dist_to_circle;
		}
		else
		{
			speed_good = false;
		}
		
		// Compute radial speed
		speed_radial = attractiveness * dist_to_circle;
		if(speed_radial > cruise_speed)
		{
			speed_radial = cruise_speed;
		}
		
		// Compute tangential direction (using cross-product)
		float tan_to_circle[3] = { -rel_pos[Y],
									rel_pos[X],
									0.0f };
		float tan_norm = vectors_norm(tan_to_circle);
		float rotation_direction = (radius >= 0) ? 1.0f : -1.0f;	// 1: Clockwise		-1: Counter-clockwise
		if(tan_norm >= 0.001f)										// Just to be 100% sure
		{
			direction_tangential[X] = rotation_direction*tan_to_circle[X] / tan_norm;
			direction_tangential[Y] = rotation_direction*tan_to_circle[Y] / tan_norm;
			direction_tangential[Z] = 0.0f;
		}
		else
		{
			speed_good = false;
		}
		
		// Compute tangential speed
		speed_tangential = cruise_speed/(1 + attractiveness2*horiz_dist_to_circle);

		// Compute overall speed
		vector[X] = direction_radial[X] 	* speed_radial +
					direction_tangential[X] * speed_tangential;
		vector[Y] = direction_radial[Y] 	* speed_radial +
					direction_tangential[Y] * speed_tangential;
		vector[Z] = direction_radial[Z] 	* speed_radial +
					direction_tangential[Z] * speed_tangential;
		
		// Normalize the whole velocity vector to cruise speed
		float velocity_norm = vectors_norm(vector);
		vector[X] = cruise_speed * vector[X] / velocity_norm;
		vector[Y] = cruise_speed * vector[Y] / velocity_norm;
		vector[Z] = cruise_speed * vector[Z] / velocity_norm;
	}
	else
	{
		speed_good = false;
	}
	
	return speed_good;
}
