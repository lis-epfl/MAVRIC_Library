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


#ifndef VECTOR_FIELD_WAYPOINT_H_
#define VECTOR_FIELD_WAYPOINT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_waypoint_handler.h"
#include "position_estimation.h"
#include "control_command.h"
#include "ahrs.h"
#include "scheduler.h"
#include "stabilisation.h"

/**
 * \brief Vector field navigation
 */
typedef struct
{
	const mavlink_waypoint_handler_t* 	waypoint_handler;			///< Waypoint list (input)
	const position_estimation_t*		pos_est;					///< Estimated position and speed (input)
	velocity_command_t*					velocity_command;			///< Velocity command (output)
	ahrs_t*								ahrs;						///< Pointer to the ahrs module
	control_command_t*					controls_nav;
	velocity_command_mode_t				command_mode;				///< Global, local or semi-local
	float								floor_altitude;				///< Floor altitude for ground avoidance
	float								velocity_max;				///< Maximum velocity (max boundary)
} vector_field_waypoint_t;	


/**
 * \brief Vector field configuration structure
 */
typedef struct
{
	velocity_command_mode_t				command_mode;				///< Global, local or semi-local
	float								floor_altitude;				///< Floor altitude for ground avoidance
	float								velocity_max;				///< Maximum velocity (max boundary)
} vector_field_waypoint_conf_t;	


/**
 * \brief               		Initialises the attitude controller structure
 * 
 * \param 	vector_field   		Pointer to vector field structure
 * \param 	config				Pointer to configuration
 * \param 	waypoint_handler	Pointer to waypoint list (input)
 * \param 	pos_est		 		Pointer to the estimated speed and position (input)
 * \param 	velocity_command	Pointer to velocity command (output)
 */
void vector_field_waypoint_init(vector_field_waypoint_t* vector_field, const vector_field_waypoint_conf_t* config, const mavlink_waypoint_handler_t* waypoint_handler, const position_estimation_t* pos_est, velocity_command_t* velocity_command, ahrs_t* ahrs, control_command_t* controls_nav);


/**
 * \brief               	Main update function
 * 
 * \param 	vector_field    Pointer to data structure
 *
 * \return	Task result, currently only TASK_RUN_SUCCESS
 */
task_return_t vector_field_waypoint_update(vector_field_waypoint_t* vector_field);

/**
 * \brief 		Vector field for circular waypoint
 * 
 * \details 	Computes a 3D velocity vector guiding the MAV around a circular waypoint
 * 				 
 * \param 		rel_pos 		Current relative position of the MAV to the waypoint (input)
 * \param 		attractiveness	Weight given to this object (input)
 * \param 		cruise_speed	Nominal speed around the circle in m/s (input)
 * \param 		radius			Radius of the circle to follow (in m)
 * \param 		vector			Velocity command vector (output)
 */
void vector_field_circular_waypoint(const float rel_pos[3], const float attractiveness, const float cruise_speed, const float radius, float vector[3]);

#ifdef __cplusplus
}
#endif

#endif /* VECTOR_FIELD_WAYPOINT_H_ */