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
 * \file analog_monitor.c
 *
 * The driver for the analog monitor module
 */ 


#include "analog_monitor.h"
#include "adc_int.h"

#define CONV_FACTOR_2 1.0f				///< Conversion factor for the analog channel 2
#define CONV_FACTOR_3 1.0f				///< Conversion factor for the analog channel 3
#define CONV_FACTOR_4 1.0f				///< Conversion factor for the analog channel 4
#define CONV_FACTOR_5 1.0f				///< Conversion factor for the analog channel 5
#define CONV_FACTOR_6 0.00155f			///< Conversion factor for the analog channel 6: 6V
#define CONV_FACTOR_7 0.00155f			///< Conversion factor for the analog channel 7: 5V_ANA
#define CONV_FACTOR_10 -0.00265f		///< Conversion factor for the analog channel 10: Input
#define CONV_FACTOR_11 -0.00265f		///< Conversion factor for the analog channel 11: Battery
#define CONV_FACTOR_12 -0.00025f		///< Conversion factor for the analog channel 12: Sonar
#define CONV_FACTOR_13 -1.0f			///< Conversion factor for the analog channel 13

/**
 * \brief Declare an array containing the conversion factor for each analog channel 
 */
const float CONV_FACTOR[MONITOR_CHANNELS] = 
{
	CONV_FACTOR_2,
	CONV_FACTOR_3,
	CONV_FACTOR_4,
	CONV_FACTOR_5,
	CONV_FACTOR_6,
	CONV_FACTOR_7,
	CONV_FACTOR_10,
	CONV_FACTOR_11,
	CONV_FACTOR_12,
	CONV_FACTOR_13
};

/**
 * \brief Trigger the analog monitor
 * Start sampling
 */
void trigger_analog_monitor(void);

/**
 * \brief Compute the average of the analog channels
 */
float analog_compute_avg(analog_monitor_t* analog_monitor, analog_rails_t rail);

void init_analog_monitor(analog_monitor_t* analog_monitor) 
{
	///< Init buffer and avg outputs
	for (int i = 0; i < MONITOR_CHANNELS; ++i)
	{
		///< Init buffer
		for (int j = 0; j < MONITOR_SAMPLES; ++j)
		{
			analog_monitor->buffer[i][j] = 0;
		}
		///< Init avg outputs
		analog_monitor->avg[i] = 0;
	}

	// Init desired ADC pin
	Init_ADCI(100000, ADCIFA_REF06VDD);

	if (analog_monitor->enable[ANALOG_RAIL_2])
	{
		adc_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_2], 
							AVR32_ADCIFA_INP_ADCIN2, 
							AVR32_ADCIFA_INN_GNDANA, 
							ADCIFA_SHG_1);  
	}
	if (analog_monitor->enable[ANALOG_RAIL_3])
	{
		adc_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_3], 
							AVR32_ADCIFA_INP_ADCIN3, 
							AVR32_ADCIFA_INN_GNDANA, 
							ADCIFA_SHG_1);  
	}
	if (analog_monitor->enable[ANALOG_RAIL_4])
	{
		adc_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_4], 
							AVR32_ADCIFA_INP_ADCIN4, 
							AVR32_ADCIFA_INN_GNDANA, 
							ADCIFA_SHG_1);  
	}
	if (analog_monitor->enable[ANALOG_RAIL_5])
	{
		adc_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_5], 
							AVR32_ADCIFA_INP_ADCIN5, 
							AVR32_ADCIFA_INN_GNDANA, 
							ADCIFA_SHG_1);  
	}
	if (analog_monitor->enable[ANALOG_RAIL_6])
	{
		adc_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_6], 
							AVR32_ADCIFA_INP_ADCIN6, 
							AVR32_ADCIFA_INN_GNDANA, 
							ADCIFA_SHG_1);  
	}
	if (analog_monitor->enable[ANALOG_RAIL_7])
	{
		adc_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_7], 
							AVR32_ADCIFA_INP_ADCIN7, 
							AVR32_ADCIFA_INN_GNDANA, 
							ADCIFA_SHG_1);  ///< 5V_ANALOG
	}
	if (analog_monitor->enable[ANALOG_RAIL_10])
	{
		adc_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_10], 
							AVR32_ADCIFA_INP_GNDANA, 
							AVR32_ADCIFA_INN_ADCIN10, 
							ADCIFA_SHG_1);  ///< BAT_FILTERED
	}
	if (analog_monitor->enable[ANALOG_RAIL_11])
	{
		adc_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_11], 
							AVR32_ADCIFA_INP_GNDANA, 
							AVR32_ADCIFA_INN_ADCIN11, 
							ADCIFA_SHG_1); ///< BATTERY
	}
	if (analog_monitor->enable[ANALOG_RAIL_12])
	{
		adc_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_12], 
							AVR32_ADCIFA_INP_GNDANA, 
							AVR32_ADCIFA_INN_ADCIN12, 
							ADCIFA_SHG_1); ///< Analog pin 12
	}
	if (analog_monitor->enable[ANALOG_RAIL_13])
	{
		adc_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_13], 
							AVR32_ADCIFA_INP_GNDANA, 
							AVR32_ADCIFA_INN_ADCIN13, 
							ADCIFA_SHG_1); ///< Analog pin 13
	}
}

void analog_monitor_update(analog_monitor_t* analog_monitor)
{
	for (int i = 0; i < MONITOR_CHANNELS; ++i)
	{
		if(analog_monitor->enable[i])
		{
			analog_monitor->avg[i] = analog_compute_avg(analog_monitor, i) * CONV_FACTOR[i];
		}
	}
	trigger_analog_monitor();
}

void trigger_analog_monitor(void) 
{
	ADCI_Start_Sampling(MONITOR_SAMPLES, 100,16, 4, false);
}

float analog_compute_avg(analog_monitor_t* analog_monitor, analog_rails_t rail)
{
	float out = 0.0f;
	int i;
	for (i = 0; i  <MONITOR_SAMPLES; i++)
	{
		out += (float)analog_monitor->buffer[rail][i];
	}
	out = out / MONITOR_SAMPLES;
	return out;
}