/* MAVLink adapter header */
#ifndef MAVLINK_BRIDGE_H
#define MAVLINK_BRIDGE_H
 
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
//#define NATIVE_BIG_ENDIAN  /moved to config 
#include "conf_platform.h"
#include "mavlink/include/mavlink_types.h"

mavlink_system_t mavlink_system;
mavlink_system_t mavlink_mission_planner;

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the
 
   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
 
   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
 
#endif /* MAVLINK_BRIDGE_H */
