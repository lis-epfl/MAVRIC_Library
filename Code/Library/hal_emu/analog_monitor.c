/*
 * analog_monitor.c
 *
 * Created: 12/10/2013 23:54:29
 *  Author: sfx
 */ 

#include "analog_monitor.h"
#include "compiler.h"

//#include "adc_int.h"

#define MONITOR_CHANNELS 4
#define MONITOR_SAMPLES 10

int16_t monitor_buffer[MONITOR_CHANNELS *MONITOR_SAMPLES];

void analog_monitor_init() {
	
}

void trigger_analog_monitor() {
	int32_t i;
	for (i=0; i<MONITOR_SAMPLES*MONITOR_CHANNELS; i++) {
		monitor_buffer[i]=0;
	}
}

float get_monitored_avg(int32_t channel) {
	float out=0.0;
	int32_t i;
	for (i=0; i<MONITOR_SAMPLES; i++) {
		out+=(float)monitor_buffer[channel + i*MONITOR_CHANNELS];
	}
	out=out / MONITOR_SAMPLES;
	return out;
}

float get_battery_rail(){
	return -CONV_FACTOR_BAT * get_monitored_avg(3);
}

float get_internal_rail(){
	return -CONV_FACTOR_INT * get_monitored_avg(2);
}


float get_6V_analog_rail(){
	return CONV_FACTOR_6V * get_monitored_avg(0);
}


float get_5V_analog_rail(){
	return CONV_FACTOR_5V * get_monitored_avg(1);
}

