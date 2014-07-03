/**
 *  Waypoint navigation controller  
 *
 * The MAV'RIC Framework
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the MAV'RIC Framework.
 */


#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "qfilter.h"
#include "mavlink_waypoint_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief initialization 
 */
void init_nav(void);

/**
 * \brief navigates the robot towards waypoint
 * \param waypoint_input destination waypoint in local coordinate system
 */
void run_navigation(local_coordinates_t waypoint_input);

/**
 * \brief computes the relative position and distance to the given way point
 * \param waypointPos Local coordinates of the waypoint
 * \param rel_pos Array to store the relative 3D position of the waypoint 
 * \return Distance to waypoint squared
 */
float set_rel_pos_n_dist2wp(float waypointPos[], float rel_pos[]);

/**
 * \brief Sets the Robot speed to reach waypoint
 * \param rel_pos Relative position of the waypoint
 * \param dist2wpSqr Squared of distance to waypoint
 */
void set_speed_command(float rel_pos[], float dist2wpSqr);

#ifdef __cplusplus
}
#endif

#endif // NAVIGATION_H_