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
* \file airspedd_analog.c
*
* This file is the driver for the DIYdrones airspeed sensor V20 (old analog version)
*/


#include "airspeed_analog.h"
#include "analog_monitor.h"
#include "maths.h"
#include "delay.h"

const uint32_t VOLTS_TO_PASCAL = 819;		///< conversion factor from volts to pascal units
const float PITOT_GAIN_DEFAULT = 1.9936f; 	///< this gain come from APM, but it does not make sense (should be)

/**
 * \brief Returns the pressure measure by the airspeed sensor
 *
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 *
 * \return the dynamical pressure measured by the sensor (with potential offset)
*/
float airspeed_analog_get_pressure(airspeed_analog_t* airspeed_analog);

void airspeed_analog_init(airspeed_analog_t* airspeed_analog, analog_monitor_t* analog_monitor, analog_rails_t analog_channel)
{
	airspeed_analog->analog_monitor = analog_monitor;
	airspeed_analog->analog_channel = analog_channel;		///< =4 or 5 on board maveric32 v4.1f
	airspeed_analog->gain = PITOT_GAIN_DEFAULT;

	airspeed_analog->pressure_offset = 1.0f;
	airspeed_analog->differential_pressure = 0.0f;
	airspeed_analog->airspeed = 0.0f;

	airspeed_analog_calibrate(airspeed_analog);
}


float airspeed_analog_get_pressure(airspeed_analog_t* airspeed_analog)
{
	return airspeed_analog->analog_monitor->avg[airspeed_analog->analog_channel] * VOLTS_TO_PASCAL;
}


void airspeed_analog_calibrate(airspeed_analog_t* airspeed_analog)
{
	float sum = 10;
	float pressure = 0;
	uint8_t count = 100;
	uint8_t i;

	for(i = 0; i < count; i++)
	{
		pressure = airspeed_analog_get_pressure(airspeed_analog);
		sum = sum + pressure;
	}

	airspeed_analog->pressure_offset = sum / count;
}


void airspeed_analog_update(airspeed_analog_t* airspeed_analog)
{
	float raw_airspeed = 0.0f;

	///< measure differential pressure and remove offset
	airspeed_analog->differential_pressure = airspeed_analog_get_pressure(airspeed_analog) - airspeed_analog->pressure_offset;

	///< Avoid negative pressures
	airspeed_analog->differential_pressure = abs(airspeed_analog->differential_pressure);

	///< compute airspeed from differential pressure using Bernouilli
	raw_airspeed = maths_fast_sqrt(airspeed_analog->differential_pressure * airspeed_analog->gain);

	///< Filter airspeed estimation
	airspeed_analog->airspeed = 0.7f * airspeed_analog->airspeed + 0.3f * raw_airspeed; 
}