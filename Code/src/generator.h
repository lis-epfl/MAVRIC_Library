/*
 * generator.h
 * utilities for the DAC function generator
 *
 * Created: 05/12/2011 19:11:23
 *  Author: Felix Schill
 */ 


#ifndef GENERATOR_H_
#define GENERATOR_H_
#include <asf.h>

#define GENERATOR_BUFFER_SIZE 512

// initialise the DAC buffer to a triangle
void init_dac_buffer_triangle();

// initialise the DAC buffer to a sine
void init_dac_buffer_sine();

// generator function to be used as callback with ADC/DAC library
// this function generates a ramp directly without a buffer
// as this is called from an interrupt, it should be very fast and non-blocking
int16_t ramp_generator(int32_t index);

// generator function to be used as callback with ADC/DAC library
// this function can generate arbitrary waves based on a lookup buffer
int16_t arbitrary_generator(int32_t index);


#endif 