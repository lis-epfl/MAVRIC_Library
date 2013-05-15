/*
 * radar_module.h
 *
 * Created: 22/04/2013 16:38:00
 *  Author: sfx
 */ 


#ifndef RADAR_MODULE_DRIVER_H_
#define RADAR_MODULE_DRIVER_H_

typedef struct {
	float velocity;
	float amplitude;
	long timestamp;
} radar_target;


void init_radar_modules();

void read_radar();

radar_target* get_radar_main_target();


#endif /* RADAR_MODULE_H_ */