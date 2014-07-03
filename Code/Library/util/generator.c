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
 * \file generator.c
 * 
 * Analog signal generator
 */


#include "generator.h"
#include "math.h"
#include "ads1274.h"


// buffer for DAC output (arbitrary waveform for function generator)
static volatile uint16_t dac_function_buffer[GENERATOR_BUFFER_SIZE];	// TODO: This should not be instanciated by default


void init_dac_buffer_triangle() 
{
	int i;
	for (i=0; i<GENERATOR_BUFFER_SIZE/2; i++) 
	{
		dac_function_buffer[i] = (uint16_t)((4095.0f / (float)(GENERATOR_BUFFER_SIZE / 2))*(float)i);
	}

	for (; i<GENERATOR_BUFFER_SIZE; i++) 
	{
		dac_function_buffer[i] = (uint16_t)(4095.0f - (4096.0f / (float)(GENERATOR_BUFFER_SIZE / 2)) * (float)(i-GENERATOR_BUFFER_SIZE / 2));
	}
}


void init_dac_buffer_sine() 
{
	int i;
	for (i=0; i<GENERATOR_BUFFER_SIZE; i++) 
	{
		dac_function_buffer[i]=(uint16_t)(4095.0f * (0.5f + 0.5f * sin((float)10 * i * 2.0f * M_PI / (float)GENERATOR_BUFFER_SIZE)));
	}
}


int16_t ramp_generator(int32_t index) 
{
	return index * 4096 / ADC_BUFFER_SIZE;
}

 
int16_t arbitrary_generator(int32_t index) 
{
	return dac_function_buffer[index * GENERATOR_BUFFER_SIZE / ADC_BUFFER_SIZE];
}
