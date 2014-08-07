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
* \file state_from_remote.h
*
* This module take remote channels as input and returns desired state and mode as output
*/

#ifndef STATE_FROM_REMOTE_H_
#define STATE_FROM_REMOTE_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "state.h"
#include "remote.h"


typedef struct
{
	const remote_t* remote;

} state_from_remote_t;


#ifdef __cplusplus
	}
#endif

#endif