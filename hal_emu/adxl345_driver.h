/*
 * adxl345_driver.h
 *
 * Created: 19/05/2012 00:29:41
 *  Author: sfx
 */ 


#ifndef ADXL345_DRIVER_H_
#define ADXL345_DRIVER_H_

#include <stdint.h>

#define ACC_X 0
#define ACC_Y 1
#define ACC_Z 2


#define ADXL_ALT_SLAVE_ADDRESS 0x53  ///adxl345

typedef struct{
	uint8_t raw_data[6];
	int16_t axes[3];
} acc_data_t; 



void adxl345_driver_init(void);

acc_data_t* adxl345_driver_get_acc_data(void);

void adxl345_driver_init_slow(void);

acc_data_t* adxl345_driver_get_acc_data_slow(void);





#endif /* ADXL345_DRIVER_H_ */