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

#include "mavlink_waypoint_handler.h"


/**
 * \brief					Initialization 
 */
void navigation_init(void);


/**
 * \brief					Navigates the robot towards waypoint
 *
 * \param	waypoint_input	Destination waypoint in local coordinate system
 */
void navigation_run(local_coordinates_t waypoint_input);


#ifdef __cplusplus
}
#endif

#endif // NAVIGATION_H_