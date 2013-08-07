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

static  i2c_schedule_event gyro_event;

#define CONFIG_POWER_ADDRESS 0x2D

#define SENSOR_REG_ADDRESS 0x32



uint8_t default_configuration[2] ={
CONFIG_POWER_ADDRESS, 8};

void init_adxl345(void) {
	

	gyro_event.callback=0;
	gyro_event.repetition_rate_ms=5;
	gyro_event.trigger_next_event=-1;

	gyro_event.config.slave_address=ADXL_ALT_SLAVE_ADDRESS;
	gyro_event.config.direction=I2C_WRITE;
	gyro_event.config.read_data=&acc_outputs.raw_data;
	gyro_event.config.read_count=6;
	gyro_event.config.write_data=&default_configuration;
	gyro_event.config.write_count=2;
	gyro_event.config.i2c_speed=400000;
	
	i2c_add_request(0, &gyro_event);
	i2c_trigger_request(0, gyro_event.schedule_slot);
	
	
	gyro_event.config.direction=I2C_WRITE1_THEN_READ;
	gyro_event.config.write_then_read_preamble=SENSOR_REG_ADDRESS;
	gyro_event.config.read_data=&acc_outputs;
	gyro_event.config.read_count=6;
	i2c_change_request(0, &gyro_event);
	i2c_trigger_request(0, gyro_event.schedule_slot);
	/**/
}

void init_adxl345_slow(void) {
	static twim_options_t twi_opt= {
		.pba_hz=64000000, 
		.speed = 100000,
		.chip = ADXL_ALT_SLAVE_ADDRESS, 
		.smbus=false
	};

	twim_master_init(&AVR32_TWIM0, &twi_opt);
	twim_write(&AVR32_TWIM0, (uint8_t*)&default_configuration, 2, ADXL_ALT_SLAVE_ADDRESS, false);
}

acc_data* get_acc_data(void) {
	i2c_trigger_request(0, gyro_event.schedule_slot);
	return &acc_outputs;
}

acc_data* get_acc_data_slow(void) {
	int i;
	gyro_event.config.write_then_read_preamble=SENSOR_REG_ADDRESS;
	twim_write(&AVR32_TWIM0, (uint8_t*) &gyro_event.config.write_then_read_preamble, 1, ADXL_ALT_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&acc_outputs, 6, ADXL_ALT_SLAVE_ADDRESS, false);
	
	for (i=0; i<3; i++) {
		acc_outputs.axes[i]=(int16_t)(acc_outputs.raw_data[2*i])+(int16_t)(acc_outputs.raw_data[2*i+1]<<8);
	}			
	return &acc_outputs;
}