// Copyright (C) 2014  Julien Lecoeur

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/**
 * Driver for the DIYdrones airspeed sensor V20 (old analog version)
 */

#include "airspeed_analog.h"
#include "analog_monitor.h"
#include "maths.h"
#include "delay.h"

const uint32_t VOLTS_TO_PASCAL = 819;
const float PITOT_GAIN_DEFAULT = 1.9936; 	// this gain come from APM, but it does not make sense (should be)


float airspeed_analog_get_pressure(airspeed_analog_t* airspeed_analog);


void airspeed_analog_init(airspeed_analog_t* airspeed_analog, analog_rails_t analog_channel)
{
	airspeed_analog->analog_channel = analog_channel;		// =4 or 5 on board maveric32 v4.1
	airspeed_analog->gain = PITOT_GAIN_DEFAULT;

	airspeed_analog->pressure_offset = 1.0;
	airspeed_analog->differential_pressure = 0.0;
	airspeed_analog->airspeed = 0.0;

	airspeed_analog_calibrate(airspeed_analog);
}


float airspeed_analog_get_pressure(airspeed_analog_t* airspeed_analog)
{
	return analog_get_avg(airspeed_analog->analog_channel) * VOLTS_TO_PASCAL;
}


void airspeed_analog_calibrate(airspeed_analog_t* airspeed_analog)
{
	float sum = 10;
	float pressure = 0;
	uint8_t count = 100;
	uint8_t i;

	for(i=0; i<count; i++)
	{
		pressure = airspeed_analog_get_pressure(airspeed_analog);
		// pressure = 300000;
		sum = sum + pressure;
		delay_ms(1);
	}

	airspeed_analog->pressure_offset = sum / count;
	// airspeed_analog->pressure_offset = sum;
}


void airspeed_analog_update(airspeed_analog_t* airspeed_analog)
{
	float raw_airspeed = 0.0;

	// measure differential pressure and remove offset
	airspeed_analog->differential_pressure = airspeed_analog_get_pressure(airspeed_analog) - airspeed_analog->pressure_offset;

	// Avoid negative pressures
	airspeed_analog->differential_pressure = abs(airspeed_analog->differential_pressure);

	// compute airspeed from differential pressure using Bernouilli
	raw_airspeed = fast_sqrt(airspeed_analog->differential_pressure * airspeed_analog->gain);

	// Filter airspeed estimation
	airspeed_analog->airspeed = 0.7f * airspeed_analog->airspeed + 0.3f * raw_airspeed; 
}