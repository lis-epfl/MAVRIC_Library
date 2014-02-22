/*
 * radar_module.h
 *
 * Created: 22/04/2013 16:38:00
 *  Author: sfx
 */ 


#ifndef RADAR_MODULE_DRIVER_H_
#define RADAR_MODULE_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float velocity;
	float amplitude;
	long timestamp;
} radar_target;


void init_radar_modules(void);

void read_radar(void);

radar_target* get_radar_main_target(void);


#ifdef __cplusplus
}
#endif

#endif /* RADAR_MODULE_H_ */