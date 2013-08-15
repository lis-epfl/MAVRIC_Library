/*
 * itg3200_driver.h
 *
 * Created: 18/05/2012 17:51:08
 *  Author: sfx
 */ 


#ifndef ITG3200_DRIVER_H_
#define ITG3200_DRIVER_H_
#include "compiler.h"

#define GY_X 0
#define GY_Y 1
#define GY_Z 2

#define ITG3200_SLAVE_ADDRESS 0b01101000

typedef struct{
	int16_t temperature;
	int16_t axes[3];
} gyro_data; 



void init_itg3200(void);
void reconfigure_gyro(void);

gyro_data* get_gyro_data(void);

void init_itg3200_slow(void);

gyro_data* get_gyro_data_slow(void);


#endif /* ITG3200_DRIVER_H_ */