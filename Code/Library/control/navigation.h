/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file navigation.h
 * 
 * Waypoint navigation controller 
 */


#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "qfilter.h"
#include "mavlink_waypoint_handler.h"

/**
 * \brief					Initialization 
 */
void init_nav(void);

/**
 * \brief					Navigates the robot towards waypoint
 *
 * \param	waypoint_input	Destination waypoint in local coordinate system
 */
void run_navigation(local_coordinates_t waypoint_input);

/**
 * \brief					Computes the relative position and distance to the given way point
 *
 * \param	waypointPos		Local coordinates of the waypoint
 * \param	rel_pos			Array to store the relative 3D position of the waypoint
 *
 * \return					Distance to waypoint squared
 */
float set_rel_pos_n_dist2wp(float waypointPos[], float rel_pos[]);

/**
 * \brief					Sets the Robot speed to reach waypoint
 *
 * \param	rel_pos			Relative position of the waypoint
 * \param	dist2wpSqr		Squared of distance to waypoint
 */
void set_speed_command(float rel_pos[], float dist2wpSqr);

#ifdef __cplusplus
}
#endif

#endif // NAVIGATION_H_