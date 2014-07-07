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

#include "mavlink_stream.h"

typedef struct {
	float velocity;
	float amplitude;
	int32_t timestamp;
} radar_target;


void radar_module_init(void);

void radar_module_read(void);

mavlink_radar_tracked_target_t* radar_module_get_main_target(void);


#ifdef __cplusplus
}
#endif

#endif /* RADAR_MODULE_H_ */