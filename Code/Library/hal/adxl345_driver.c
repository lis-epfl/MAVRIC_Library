/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file adxl345_driver.c
*
* This file is the Driver for the ADXL345 accelerometer
*/


#include "adxl345_driver.h"
#include "i2c_driver_int.h"
#include "print_util.h"
#include "twim.h"

static volatile acc_data acc_outputs;				///< Declare an object containing accelerometer's data

#define CONFIG_POWER_ADDRESS 0x2D					///< Address of the power configuration register

#define SENSOR_REG_ADDRESS 0x32						///< Address of the accelerometer register
#define DATA_SETTING_ADDRESS 0x31					///< Address of the data setting register
enum {RANGE_2G, RANGE_4G, RANGE_8G, RANGE_16G};		///< Define the different range in which you could use the accelerometer
#define FULL_RES 0b1000								///< Define the full resolution for the output of the accelerometer

uint8_t default_configuration[2] ={CONFIG_POWER_ADDRESS, 8};	///< default configuration of the accelerometer

uint8_t data_configuration[2] ={DATA_SETTING_ADDRESS, FULL_RES | RANGE_16G};	///< configuration of the output data

void adxl345_driver_init_slow(void) 
{
	static twim_options_t twi_opt = 
	{
		.pba_hz = 64000000, 
		.speed	= 400000,
		.chip	= ADXL_ALT_SLAVE_ADDRESS, 
		.smbus	= false
	};

	twim_master_init(&AVR32_TWIM0, &twi_opt);
	twim_write(&AVR32_TWIM0, (uint8_t*)&default_configuration, 2, ADXL_ALT_SLAVE_ADDRESS, false);
	twim_write(&AVR32_TWIM0, (uint8_t*)&data_configuration, 2, ADXL_ALT_SLAVE_ADDRESS, false);
}

acc_data* adxl345_driver_get_acc_data_slow(void) 
{
	int32_t i;
	uint8_t write_then_read_preamble=SENSOR_REG_ADDRESS;
	
	twim_write(&AVR32_TWIM0, (uint8_t*)&write_then_read_preamble, 1, ADXL_ALT_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&acc_outputs, 6, ADXL_ALT_SLAVE_ADDRESS, false);
	
	for (i = 0; i < 3; i++) 
	{
		acc_outputs.axes[i] = (int16_t)(acc_outputs.raw_data[2 * i]) + (int16_t)(acc_outputs.raw_data[2 * i + 1] << 8);
	}
				
	return (acc_data*)&acc_outputs;
}