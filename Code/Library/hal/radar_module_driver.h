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


void init_radar_modules(void);

void read_radar(void);

mavlink_radar_tracked_target_t* get_radar_main_target(void);


#ifdef __cplusplus
}
#endif

#endif /* RADAR_MODULE_H_ */