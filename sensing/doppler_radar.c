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
 * \file doppler_radar.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief The doppler effect computed by the radar
 *
 ******************************************************************************/

 
#include "doppler_radar.h"
#include "print_util.h"
#include "math.h"
#include "time_keeper.h"

static volatile uint32_t even_odd2;

A_ALIGNED dsp16_complex_t vect_outputI[ RADAR_BUFFER_SIZE * 2];	// Variable pour le DSP
A_ALIGNED dsp16_complex_t vect_outputQ[ RADAR_BUFFER_SIZE * 2];
dsp16_t vect_inputI[ RADAR_BUFFER_SIZE];
//dsp16_t vect_inputQ[ ADCI_BUFFER_SIZE];
//dsp16_t vect_realI[ ADCI_BUFFER_SIZE];
//dsp16_t vect_imagI[ ADCI_BUFFER_SIZE];
//dsp16_t vect_realQ[ ADCI_BUFFER_SIZE];
//dsp16_t vect_imagQ[ ADCI_BUFFER_SIZE];
	
//dsp16_t vect_realpow[ADCI_BUFFER_SIZE];
//dsp16_t vect_imagpow[ADCI_BUFFER_SIZE];

int32_t fft_amp[RADAR_BUFFER_SIZE];

radar_target_t main_target;


int32_t alpha = 12;		//constant ( * 100) for the lowpass filtering of amplitude
int32_t input_max = 0;
int32_t input_min = 0;
int32_t rms = 0;
int32_t amplitude = 0;
int32_t amplitude2 = 0;
int32_t amp_old = 0;
int32_t mean_amp = 0;
int32_t frequency = 0;
int32_t speed = 0;
int32_t speed_old = 0;
int32_t speed2 = 0;
int32_t index2 = 0;
int32_t amp_in = 0;
int32_t taille = 0;
int32_t direction = 0;
int32_t f_v_factor = Sampling_frequency / RADAR_BUFFER_SIZE;		//Conversion factor to compute speed

uint32_t time1, time2,time_result; //time1 - time2 = time_result for measure

	
void calculate_radar(dsp16_t i_buffer[], dsp16_t q_buffer[]) 
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t counter = 0;
	int32_t reading_status = 3;
	int32_t *c = 0;
	int32_t read_value = 0;
	int32_t index = 0;

	dsp16_trans_realcomplexfft(vect_outputI, i_buffer, FFT_POWER);  //WARNING !! If you change the buffer size you need to change the base 2 power size
	dsp16_trans_realcomplexfft(vect_outputQ, q_buffer, FFT_POWER);  //WARNING !! If you change the buffer size you need to change the base 2 power size
			
	for (i = 0;i< RADAR_BUFFER_SIZE;i++)
	{				
		fft_amp[i] = maths_fast_sqrt(SQR(vect_outputI[i].real) + SQR(vect_outputI[i].imag));
	}

	//Find maximum of FFT and corresponding frequency
	//time1 = time_keeper_get_micros();
	amplitude = 0;
	index = 0;
	for(i = 1;i < RADAR_BUFFER_SIZE / 2 - 1; i++) //ignore the element 0 (low frequency noise)
	{
		if(fft_amp[i] > amplitude)
		{
			amplitude = fft_amp[i];
			index = i;			//find index of max
		}
	}
			
	//Let's find the second maximum peak
	amplitude2 = 0;
	index2 = 0;
	for(i = 1;i < RADAR_BUFFER_SIZE / 2-1;i++)
	{
		if(fft_amp[i] > amplitude2 && i != index)
		{
			amplitude2 = fft_amp[i];
			index2 = i;	
		}
	}
			
	//Don't need to test the following with index2 because index corresponds to the absolute maximum anyway
	if(index > 1)
	{
		mean_amp = (2 * fft_amp[index] + fft_amp[index + 1] + fft_amp[index - 1]) / 4; //Average on the three peaks in Fourier domain
	}
	else
	{
		mean_amp = (fft_amp[index] + fft_amp[index + 1]) / 2; //Same average	
	}
			
	if(mean_amp < THRESHOLD)
	{
		mean_amp = 0;
		index = 0;
		index2 = 0;
		amp_old = 0;
		speed = 0;
	}
	else
	{
			//lowpass exponential filtering applied (alpha =0.75f so we multiply by 100, do the operations then divide by 100)
		if(index > 1)
		{
			mean_amp = (alpha * ((fft_amp[index] + fft_amp[index + 1] + fft_amp[index - 1]) / 3) + (filter_conversion - alpha) * amp_old) / filter_conversion;
			amp_old = mean_amp;	//update the value of the previous amplitude for next iteration
		}
		else
		{
			mean_amp = (alpha * ((fft_amp[index] + fft_amp[index + 1]) / 2) + (filter_conversion - alpha) * amp_old) / filter_conversion;
			amp_old = mean_amp;
		}
		//Factor 1000 for speed to avoid decimal value
		//The true speed is (speed) / 100 (m / s) or interpret it as cm / s
			
		frequency = f_v_factor * 10 * (index);
		speed = 1000 * (3 * powf(10,8) * frequency / (2 * 2415 * pow(10,9)));	//compute speed (*0.01f because of Fsample and *10 because of freq)
			
		frequency = f_v_factor * 10 * (index2);
		speed2 = 1000 * (3 * powf(10,8) * frequency / (2 * 2415 * pow(10,9)));	//compute speed (*0.01f because of Fsample and *10 because of freq)											
			
		//Let's find the speed that is closer to the previous one (to avoid high frequency changes for the speed)
					
		if(abs(speed - speed_old) <= abs(speed2 - speed_old))
		{
			speed_old = speed;			//update the old speed
	
		}
		else if(abs(speed - speed_old) > abs(speed2 - speed_old)) // in this case the speed is the one corresponding to the second peak
		{
			speed_old = speed2;		//update the old speed
			speed = speed2;	
			index = index2;
					
			if(index > 1)
			{
				mean_amp = (alpha * ((fft_amp[index2] + fft_amp[index2 + 1] + fft_amp[index2 - 1]) / 3) + (filter_conversion - alpha) * amp_old) / filter_conversion;
				amp_old = mean_amp;
			}					
			else
			{
				//mean_amp = (vect_inputQ[index2] + vect_inputQ[index2 + 1]) / 2;	
				mean_amp = (alpha * ((fft_amp[index2] + fft_amp[index2 + 1]) / 2) + (filter_conversion - alpha) * amp_old) / filter_conversion;
				amp_old = mean_amp;
			}					
		}
	}

	//Find the direction of motion by doing vector product between I and Q channel
	if(vect_outputI[index].real * vect_outputQ[index].imag - vect_outputI[index].imag * vect_outputQ[index].real < 0)
	{
		direction = -1;
	}
	else if(speed == 0)
	{
		direction = 0;
	}
	else
	{	
		direction = 1;
	}
	
	for (i = 0; i < RADAR_BUFFER_SIZE; i++) 
	{
		if(vect_outputI[index].real * vect_outputQ[index].imag - vect_outputI[index].imag * vect_outputQ[index].real < 0)
		{
			fft_amp[i] = -fft_amp[i];
		}
	}
	
	time2 = time_keeper_get_micros();
	time_result = time2 - time1;
	time1 = time_keeper_get_micros();

	main_target.velocity = direction * speed / 100.0f;
	//main_target.velocity = speed / 100.0f;
	main_target.amplitude = amplitude;

	amplitude = 0;
	amplitude2 = 0;
	index = 0;
	index2 = 0;
	input_min = 0;
	input_max = 0;
	rms = 0;
	mean_amp = 0;
	read_value = 0;
}

radar_target_t* get_tracked_target()
{
	return &main_target;
}

int32_t* get_raw_FFT() 
{
	return &fft_amp;
}
