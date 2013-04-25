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


#define THRESHOLD 3000
//#define Sampling_frequency 23437
#define Sampling_frequency 15625
#define filter_conversion 100


void calculate_radar();

radar_target* get_tracked_target();

#endif /* DOPPLER_RADAR_H_ */