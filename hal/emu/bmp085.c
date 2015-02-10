/*
 * bmp085.c
 *
 * Created: 12/03/2013 22:14:46
 *  Author: sfx
 */ 
#include "bmp085.h"

//#include "twim.h"
//#include "delay.h"

#include "math.h"

barometer_t pressure_outputs;

void bmp085_init(){
	bmp085_init_slow();
}

void bmp085_init_slow(){
	
}



barometer_t* bmp085_update(float offset){
		
		return &pressure_outputs;
}

bool bmp085_new_valid_barometer(uint32_t *time_prev_barometer) {
		if (*time_prev_barometer < pressure_outputs.last_update) 
	{
		*time_prev_barometer = pressure_outputs.last_update;
		return true;
	}else{
		return false;
	}
}