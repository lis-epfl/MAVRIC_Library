/*
 * adxl345_driver.h
 *
 * Created: 19/05/2012 00:29:41
 *  Author: sfx
 */ 


#ifndef ADXL345_DRIVER_H_
#define ADXL345_DRIVER_H_

#include "compiler.h"

#define ACC_X 0
#define ACC_Y 1
#define ACC_Z 2


#define ADXL_ALT_SLAVE_ADDRESS 0x53  ///adxl345

typedef struct{
	uint8_t raw_data[6];
	int16_t axes[3];
} acc_data; 



void init_adxl345();

acc_data* get_acc_data();

void init_adxl345_slow();

acc_data* get_acc_data_slow();





#endif /* ADXL345_DRIVER_H_ */