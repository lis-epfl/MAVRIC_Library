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
 * \file mavlink_bridge.h
 * 
 * MAVLink adapter header
 */


#ifndef MAVLINK_BRIDGE_H
#define MAVLINK_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "conf_platform.h"
#include "stdint.h"

#include "mavlink/include/mavlink_types.h"

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

// Structures that stores the communication settings of this system: DO NOT (re)move
extern mavlink_system_t mavlink_system;
extern mavlink_system_t mavlink_mission_planner;

/**
 * \brief Send one char (uint8_t) over a communication channel
 *
 * \param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * \param ch Character to send
 */
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_BRIDGE_H */
