/*
 * generator.c
 *
 * Created: 05/12/2011 19:13:20
 *  Author: sfx
 */ 

#include "generator.h"
#include "math.h"
#include "ads1274.h"

// buffer for DAC output (arbitrary waveform for function generator)
static volatile uint16_t dac_function_buffer[GENERATOR_BUFFER_SIZE];


// initialise the DAC buffer to a triangle
void init_dac_buffer_triangle() {
	int i;
	for (i=0; i<GENERATOR_BUFFER_SIZE/2; i++) 
	{
		dac_function_buffer[i]=(uint16_t)((4095.0/(float)(GENERATOR_BUFFER_SIZE/2))*(float)i);
	}

	for (; i<GENERATOR_BUFFER_SIZE; i++) 
	{
		dac_function_buffer[i]=(uint16_t)(4095.0-(4096.0/(float)(GENERATOR_BUFFER_SIZE/2))*(float)(i-GENERATOR_BUFFER_SIZE/2));
	}

}

// initialise the DAC buffer to a sine
void init_dac_buffer_sine() {
	int i;
	for (i=0; i<GENERATOR_BUFFER_SIZE; i++) 
	{
		dac_function_buffer[i]=(uint16_t)(4095.0*(0.5+0.5*sin((float)10*i*2.0*M_PI/(float)GENERATOR_BUFFER_SIZE)));
	}


}

// generator function to be used as callback with ADC/DAC library
// this function generates a ramp directly without a buffer
// as this is called from an interrupt, it should be very fast and non-blocking
int16_t ramp_generator(int32_t index) {
	return index*4096/ADC_BUFFER_SIZE;
}

// generator function to be used as callback with ADC/DAC library
// this function can generate arbitrary waves based on a lookup buffer
 
int16_t arbitrary_generator(int32_t index) {
	return dac_function_buffer[index*GENERATOR_BUFFER_SIZE/ADC_BUFFER_SIZE];
}
