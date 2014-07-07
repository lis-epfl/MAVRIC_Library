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
 * \file doppler_radar.h
 * 
 * The doppler effect computed by the radar
 */


#ifndef DOPPLER_RADAR_H_
#define DOPPLER_RADAR_H_

#include "radar_module_driver.h"
#include "radar_driver.h"
#include "adc_int.h"
#include "dsp.h"


#define THRESHOLD 500						///< Define the radar doppler threshold
//#define Sampling_frequency 23437			///< Define the radar doppler sampling frequency
#define Sampling_frequency 6000//15625		///< Define the radar doppler sampling frequency
#define filter_conversion 100				///< Define the radar doppler filter conversion factor
#define RADAR_BUFFER_SIZE 512				///< Define the radar doppler buffer size
#define FFT_POWER 9  //2^9 =512				///< Define the radar doppler fft power

/**
 * \brief Compute the doppler radar
 *
 * \param i_buffer the input(?) buffer
 * \param q_buffer the q filter(?) buffer
 *
 */
void calculate_radar(dsp16_t i_buffer[], dsp16_t q_buffer[]);

/**
 * \brief Return the tracked target
 *
 * \return a pointer to the radar target object
 */
radar_target* get_tracked_target();

/**
 * \brief Return the raw FFT of the radar module
 *
 * \return buffer containing the raw FFT
 */
int32_t* get_raw_FFT();

#endif /* DOPPLER_RADAR_H_ */