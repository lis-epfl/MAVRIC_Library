/*
 * doppler_radar.h
 *
 * Created: 19/04/2013 16:41:40
 *  Author: sfx
 */ 


#ifndef DOPPLER_RADAR_H_
#define DOPPLER_RADAR_H_

#include "radar_module_driver.h"
#include "radar_driver.h"
#include "adc_int.h"
#include "dsp.h"


#define THRESHOLD 500
//#define Sampling_frequency 23437
#define Sampling_frequency 3000//15625
#define filter_conversion 100
#define RADAR_BUFFER_SIZE 1024
#define FFT_POWER 10  //2^9 =512

void calculate_radar(dsp16_t i_buffer[], dsp16_t q_buffer[]);

radar_target* get_tracked_target();
int32_t* get_raw_FFT();

#endif /* DOPPLER_RADAR_H_ */