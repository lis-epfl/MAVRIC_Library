/*
 * adxl345_driver.c
 *
 * Created: 19/05/2012 00:29:28
 *  Author: sfx
 */ 

#include "adxl345_driver.h"
#include "i2c_driver_int.h"
#include "print_util.h"
#include "twim.h"

static volatile acc_data acc_outputs;


#define CONFIG_POWER_ADDRESS 0x2D

#define SENSOR_REG_ADDRESS 0x32
#define DATA_SETTING_ADDRESS 0x31
enum {RANGE_2G, RANGE_4G, RANGE_8G, RANGE_16G};
#define FULL_RES 0b1000

uint8_t default_configuration[2] ={
CONFIG_POWER_ADDRESS, 8};



uint8_t data_configuration[2] ={
DATA_SETTING_ADDRESS, FULL_RES | RANGE_16G};



void init_adxl345_slow(void) {
	static twim_options_t twi_opt= {
		.pba_hz=64000000, 
		.speed = 400000,
		.chip = ADXL_ALT_SLAVE_ADDRESS, 
		.smbus=false
	};

	twim_master_init(&AVR32_TWIM0, &twi_opt);
	twim_write(&AVR32_TWIM0, (uint8_t*)&default_configuration, 2, ADXL_ALT_SLAVE_ADDRESS, false);
	twim_write(&AVR32_TWIM0, (uint8_t*)&data_configuration, 2, ADXL_ALT_SLAVE_ADDRESS, false);
}


acc_data* get_acc_data_slow(void) {
	int i;
	uint8_t write_then_read_preamble=SENSOR_REG_ADDRESS;
	twim_write(&AVR32_TWIM0, (uint8_t*) &write_then_read_preamble, 1, ADXL_ALT_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&acc_outputs, 6, ADXL_ALT_SLAVE_ADDRESS, false);
	
	for (i=0; i<3; i++) {
		acc_outputs.axes[i]=(int16_t)(acc_outputs.raw_data[2*i])+(int16_t)(acc_outputs.raw_data[2*i+1]<<8);
	}			
	return &acc_outputs;
}