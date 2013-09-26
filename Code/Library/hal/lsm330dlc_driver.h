/*
 * itg3200_driver.h
 *
 * Created: 18/05/2012 17:51:08
 *  Author: sfx
 */ 


#ifndef LSM330DLC_DRIVER_H_
#define LSM330DLC_DRIVER_H_
#include "compiler.h"

#define GY_X 0
#define GY_Y 1
#define GY_Z 2

#define LSM330_ACC_SLAVE_ADDRESS  0b00110000
#define LSM330_GYRO_SLAVE_ADDRESS 0b11010100

typedef struct{
	int16_t temperature;
	int16_t axes[3];
} gyro_data; 

typedef struct{
	int16_t temperature;
	int16_t axes[3];
} acc_data;


void init_lsm330(void);
void reconfigure_gyro_lsm330(void);

gyro_data* get_lsm330_gyro_data(void);
acc_data* get_lsm330_acc_data(void);


#endif 