/*
 * itg3200.h
 *
 * Created: 18/05/2012 17:51:08
 *  Author: sfx
 */ 


#ifndef ITG3200_H_
#define ITG3200_H_
#include "compiler.h"

#define GY_X 0
#define GY_Y 1
#define GY_Z 2

#define ITG3200_SLAVE_ADDRESS 0b01101000

typedef struct{
	int16_t temperature;
	int16_t axes[3];
} gyroscope_t; 



void itg3200_init(void);
void itg3200_reconfigure_gyro(void);

gyroscope_t* itg3200_get_gyro_data(void);

void itg3200_init_slow(void);

gyroscope_t* itg3200_get_data_slow(void);


#endif /* ITG3200_H_ */