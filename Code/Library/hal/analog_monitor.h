/*
 * analog_monitor.h
 *
 * Created: 12/10/2013 23:47:01
 *  Author: sfx
 */ 


#ifndef ANALOG_MONITOR_H_
#define ANALOG_MONITOR_H_

#include "compiler.h"

typedef enum
{
	ANALOG_RAIL_6V,
	ANALOG_RAIL_5V,
	ANALOG_RAIL_INTERNAL,
	ANALOG_RAIL_BATTERY,
	ANALOG_RAIL_12,
	ANALOG_RAIL_13,
} analog_rails_t;

void init_analog_monitor(void);
void trigger_analog_monitor(void);

float analog_get_avg(analog_rails_t rail);

float get_6V_analog_rail(void);
float get_5V_analog_rail(void);
float get_internal_rail(void);
float get_battery_rail(void);
float get_analog_rail_12(void);
float get_analog_rail_13(void);

#endif /* ANALOG_MONITOR_H_ */