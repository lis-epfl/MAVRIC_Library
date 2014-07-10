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
 * \file radar_module_driver.h
 * 
 * The radar module driver
 */


#ifndef RADAR_MODULE_DRIVER_H_
#define RADAR_MODULE_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_stream.h"

/**
 * \brief Structure containing the radar target
 */
typedef struct 
{
	float velocity;			///< Define the velocity of the target
	float amplitude;		///< Define the amplitude signal
	int32_t timestamp;		///< Define the time stamp to use
} radar_target_t;

/**
 * \brief Initialize the radar module
 */
void radar_module_init(void);

/**
 * \brief Read the radar module
 */
void radar_module_read(void);

/**
 * \brief Return the main target
 *
 * \return a pointer to an object containing the mavlink radar tracked target
 */
mavlink_radar_tracked_target_t* radar_module_get_main_target(void);


#ifdef __cplusplus
}
#endif

#endif /* RADAR_MODULE_H_ */