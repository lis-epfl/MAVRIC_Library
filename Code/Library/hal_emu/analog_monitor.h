/*
 * analog_monitor.h
 *
 * Created: 12/10/2013 23:47:01
 *  Author: sfx
 */ 


#ifndef ANALOG_MONITOR_H_
#define ANALOG_MONITOR_H_

#define CONV_FACTOR_BAT 0.00265
#define CONV_FACTOR_INT 0.00265
#define CONV_FACTOR_6V 0.00155
#define CONV_FACTOR_5V 0.00155

void analog_monitor_init();
void trigger_analog_monitor();
float get_battery_rail();

float get_internal_rail();


float get_6V_analog_rail();


float get_5V_analog_rail();

#endif /* ANALOG_MONITOR_H_ */