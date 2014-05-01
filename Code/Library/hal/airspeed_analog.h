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

#ifndef AIRSPEED_ANALOG_H_
#define AIRSPEED_ANALOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "compiler.h"
#include "analog_monitor.h"

typedef struct {
	uint8_t analog_channel;
	float gain;
	float pressure_offset;
	float differential_pressure;
	float airspeed;
} airspeed_analog_t;


void airspeed_analog_init(airspeed_analog_t* airspeed_analog, analog_rails_t analog_channel);

void airspeed_analog_calibrate(airspeed_analog_t* airspeed_analog);

void airspeed_analog_update(airspeed_analog_t* airspeed_analog);


#ifdef __cplusplus
}
#endif

#endif /* AIRSPEED_ANALOG_H_ */