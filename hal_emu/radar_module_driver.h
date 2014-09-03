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
} radar_target_t;


void radar_module_init(void);

void radar_module_read(void);

radar_target_t* radar_module_get_main_target(void);


#endif /* RADAR_MODULE_H_ */