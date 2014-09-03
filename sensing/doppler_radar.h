/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file doppler_radar.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief The doppler effect computed by the radar
 *
 ******************************************************************************/


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
radar_target_t* get_tracked_target();

/**
 * \brief Return the raw FFT of the radar module
 *
 * \return buffer containing the raw FFT
 */
int32_t* get_raw_FFT();

#endif /* DOPPLER_RADAR_H_ */