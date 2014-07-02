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
* \file airspedd_analog.h
*
* This file is the driver for the DIYdrones airspeed sensor V20 (old analog version)
*/


#ifndef AIRSPEED_ANALOG_H_
#define AIRSPEED_ANALOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "compiler.h"
#include "analog_monitor.h"

/**
 * \brief Structure containing the analog airspeed sensor datas
*/
typedef struct {
	uint8_t analog_channel;					///< analog channel of the ADC
	float gain;								///< gain factor for the ADC
	float pressure_offset;					///< offset of the pressure sensor
	float differential_pressure;			///< true dynamical pressure (diff between pressure and offset)
	float airspeed;							///< measure airspeed
	analog_monitor_t* analog_monitor;		///< pointer to the structure of analog monitor module
} airspeed_analog_t;

/**
 * \brief Initialize the airspeed sensor
 *
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 * \param analog_monitor pointer to the structure of analog monitor module
 * \param analog_channel set which channel of the ADC is map to the airspeed sensor
 *
*/
void airspeed_analog_init(airspeed_analog_t* airspeed_analog, analog_monitor_t* analog_monitor, analog_rails_t analog_channel);

/**
 * \brief Calibrates the airspeed sensor
 * 
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 *
*/
void airspeed_analog_calibrate(airspeed_analog_t* airspeed_analog);

/**
 * \brief Updates the values in the airspeed structure
 *
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 *
*/
void airspeed_analog_update(airspeed_analog_t* airspeed_analog);


#ifdef __cplusplus
}
#endif

#endif /* AIRSPEED_ANALOG_H_ */