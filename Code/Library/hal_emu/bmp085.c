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

pressure_data_t pressure_outputs;

void bmp085_init(){
	bmp085_init_slow();
}

void bmp085_init_slow(){
	
}



pressure_data_t* bmp085_update(float offset){
		
		return &pressure_outputs;
}

bool bmp085_newValidBarometer(uint32_t *timePrevBarometer) {
		if (*timePrevBarometer < pressure_outputs.last_update) 
	{
		*timePrevBarometer = pressure_outputs.last_update;
		return true;
	}else{
		return false;
	}
}