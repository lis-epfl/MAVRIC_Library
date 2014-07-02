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
 * \file generator.h
 * 
 * Analog signal generator
 */ 


#ifndef GENERATOR_H_
#define GENERATOR_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#include <asf.h>

#define GENERATOR_BUFFER_SIZE 512


/**
 * \brief       Initialises the DAC buffer to a triangle
 */
void init_dac_buffer_triangle();

 
/**
 * \brief       Initialises the DAC buffer to a sine
 */
void init_dac_buffer_sine();


/**
 * \brief       	Generator function to be used as callback with ADC/DAC library
 * 
 * \details     	This function generates a ramp directly without a buffer
 * 					as this is called from an interrupt, it should be very fast and non-blocking
 * 
 * \param index 	Index
 * 
 * \return 			Signal  
 */
int16_t ramp_generator(int32_t index);


/**
 * \brief       	Generator function to be used as callback with ADC/DAC library
 * \details     	This function can generate arbitrary waves based on a lookup buffer
 * 
 * \param index 	Index
 * 
 * \return      	Signal
 */
int16_t arbitrary_generator(int32_t index);


#ifdef __cplusplus
}
#endif

#endif 