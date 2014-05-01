/*
 * analog_monitor.c
 *
 * Created: 12/10/2013 23:54:29
 *  Author: sfx
 */ 

#include "analog_monitor.h"

#include "adc_int.h"

#define MONITOR_CHANNELS 6
#define MONITOR_SAMPLES  10

const float CONV_FACTOR_BAT = -0.00265;
const float CONV_FACTOR_INT = -0.00265;
const float CONV_FACTOR_6V  = 0.00155;
const float CONV_FACTOR_5V  = 0.00155;
const float CONV_FACTOR_12  = -1.0;
const float CONV_FACTOR_13  = -1.0;

int16_t monitor_buffer[MONITOR_CHANNELS][MONITOR_SAMPLES];

void init_analog_monitor(void) 
{
	Init_ADCI(100000, ADCIFA_REF06VDD);
	adc_sequencer_add(monitor_buffer[0], AVR32_ADCIFA_INP_ADCIN6, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1);  // 6V
	adc_sequencer_add(monitor_buffer[1], AVR32_ADCIFA_INP_ADCIN7, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1);  // 5V_ANALOG
	adc_sequencer_add(monitor_buffer[2], AVR32_ADCIFA_INP_GNDANA, AVR32_ADCIFA_INN_ADCIN10, ADCIFA_SHG_1);  // BAT_FILTERED
	adc_sequencer_add(monitor_buffer[3], AVR32_ADCIFA_INP_GNDANA, AVR32_ADCIFA_INN_ADCIN11, ADCIFA_SHG_1); // INPUT
	adc_sequencer_add(monitor_buffer[4], AVR32_ADCIFA_INP_GNDANA, AVR32_ADCIFA_INN_ADCIN12, ADCIFA_SHG_1); // Analog pin 12
	adc_sequencer_add(monitor_buffer[5], AVR32_ADCIFA_INP_GNDANA, AVR32_ADCIFA_INN_ADCIN13, ADCIFA_SHG_1); // Analog pin 13
}

void trigger_analog_monitor(void) 
{
	ADCI_Start_Sampling(MONITOR_SAMPLES, 100,16, 4, false);
}

float get_monitored_avg(int channel) {
	float out=0.0;
	int i;
	for (i=0; i<MONITOR_SAMPLES; i++) {
		out+=(float)monitor_buffer[channel][i];
	}
	out=out / MONITOR_SAMPLES;
	return out;
}

float get_6V_analog_rail(void){
	return CONV_FACTOR_6V * get_monitored_avg(0);
}

float get_5V_analog_rail(void){
	return CONV_FACTOR_5V * get_monitored_avg(1);
}

float get_internal_rail(void){
	return CONV_FACTOR_INT * get_monitored_avg(2);
}

float get_battery_rail(void){
	return CONV_FACTOR_BAT * get_monitored_avg(3);
}

float get_analog_rail_12(void){
	return CONV_FACTOR_12 * get_monitored_avg(4);
}

float get_analog_rail_13(void){
	return CONV_FACTOR_13 * get_monitored_avg(5);
}

float analog_get_avg(analog_rails_t rail)
{
	return get_monitored_avg(rail);
}
