/*
 * adxl345_driver.c
 *
 * Created: 19/05/2012 00:29:28
 *  Author: sfx
 */ 

#include "adxl345_driver.h"
//#include "i2c_driver_int.h"
#include "print_util.h"
//#include "twim.h"

static volatile acc_data_t acc_outputs;

//static  i2c_schedule_event_t gyro_event;

#define CONFIG_POWER_ADDRESS 0x2D

#define SENSOR_REG_ADDRESS 0x32



uint8_t default_configuration[2] ={
CONFIG_POWER_ADDRESS, 8};

void adxl345_driver_init(void) {
	


}

void adxl345_driver_init_slow(void) {

}

acc_data_t* adxl345_driver_get_acc_data(void) {
	
	return &acc_outputs;
}

acc_data_t* adxl345_driver_get_acc_data_slow(void) {
	
	return &acc_outputs;
}