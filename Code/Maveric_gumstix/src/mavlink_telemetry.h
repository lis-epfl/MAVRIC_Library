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
 * \file mavlink_telemetry.h
 *
 * Definition of the messages sent by the autopilot to the ground station
 */ 


#ifndef MAVLINK_TELEMETRY_H_
#define MAVLINK_TELEMETRY_H_

#include "mavlink_stream.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief     Initialise all the mavlink streams and call the onboard parameters register
 */
void mavlink_telemetry_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_DOWN_TELEMETRY_H_ */