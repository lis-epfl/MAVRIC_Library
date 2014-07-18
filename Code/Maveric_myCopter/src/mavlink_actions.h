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
 * \file mavlink_action.h
 *
 * Definition of the tasks executed on the autopilot
 */ 


#ifndef MAVLINK_ACTIONS_H_
#define MAVLINK_ACTIONS_H_

#include "mavlink_stream.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief     Add all onboard parameters to the parameter list
 */
void mavlink_actions_add_onboard_parameters(void);

/**
 * \brief     Initialisation of the module mavlink actions
 */
void mavlink_actions_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_ACTIONS_H_ */