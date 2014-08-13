/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file conf_constants.h
 *
 * This file defines global constants
 */ 


#ifndef CONF_CONSTANT_H__
#define CONF_CONSTANT_H__

#ifdef __cplusplus
extern "C" {
#endif

// default home location (EFPL Esplanade)
#define HOME_LONGITUDE 6.566044801857777f					///< Latitude of home location
#define HOME_LATITUDE 46.51852236174565f 					///< Longitude of home location
#define HOME_ALTITUDE 400.0f								///< Altitude of home location

#define GRAVITY 9.81f				///< The gravity constant


#ifdef __cplusplus
}
#endif

#endif // CONF_CONSTANT_H__