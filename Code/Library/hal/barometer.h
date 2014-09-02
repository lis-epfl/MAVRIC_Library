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
 * \file barometer.h
 *
 * This file define the barometer
 */


#ifndef BAROMETER_H_
#define BAROMETER_H_

#include "mavlink_communication.h"

/**
 * \brief bmp085_state_t can get three different state: Idle, get Temperature or get Pressure
*/
typedef enum bmp085_state_t
{
	IDLE,				///< Idle state
	GET_TEMP,			///< Getting temperature state
	GET_PRESSURE		///< Getting pressure state
} barometer_state_t;

typedef struct
{
	uint8_t 	raw_pressure[3];		///< Raw pressure contained in 3 uint8_t
	uint8_t 	raw_temperature[2];		///< Raw temperature contained in 2 uint8_t
	
	float 		pressure;					///< Measured pressure as the concatenation of the 3 uint8_t raw_pressure
	float 		temperature;				///< Measured temperature as the concatenation of the 2 uint8_t raw_temperature
	float 		altitude;					///< Measured altitude as the median filter of the 3 last_altitudes
	float 		altitude_offset;			///< Offset of the barometer sensor for matching GPS altitude value
	float 		vario_vz;					///< Vario altitude speed
	
	float 		last_altitudes[3];		///< Array to store previous value of the altitude for low pass filtering the output
	
	uint32_t 	last_update;			///< Time of the last update of the barometer
	uint32_t 	last_state_update;		///< Time of the last state update
	barometer_state_t state;	///< State of the barometer sensor (IDLE, GET_TEMP, GET_PRESSURE)
	float 		dt;						///< Time step for the derivative
	
	const mavlink_stream_t* mavlink_stream;			///< The pointer to the mavlink stream structure
} barometer_t;

#endif /* BAROMETER_H_ */
