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
* \file i2cxl_sonar.c
*
* This file is the driver for the sonar module using i2C communication protocol
*/


#include "i2cxl_sonar.h"
#include "twim.h"
#include "delay.h"
#include "print_util.h"

const uint8_t I2CXL_DEFAULT_ADDRESS				= 0x70;		///< Address of the device
const uint8_t I2CXL_RANGE_COMMAND				= 0x51;		///< Address of the Range Command Register
const uint8_t I2CXL_CHANGE_ADDRESS_COMMAND_1	= 0xAA;		///< Address of the Change Command address Register
const uint8_t I2CXL_CHANGE_ADDRESS_COMMAND_2	= 0xA5;		///< Address of the Change Command address Register

/**
 * \brief Send range Command for the i2cxl_sonar
 *
 * \param i2c_sonar pointer to an object containing the i2cxl_sonar's data
 */
void i2cxl_send_range_command(i2cxl_sonar_t* i2c_sonar);

/**
 * \brief Get the last measurement
 *
 * \param i2c_sonar pointer to an object containing the i2cxl_sonar's data
 */
void i2cxl_get_last_measure(i2cxl_sonar_t* i2c_sonar);


void i2cxl_sonar_init(i2cxl_sonar_t* i2cxl_sonar)
{
	///< Init data_struct
	i2cxl_sonar->i2c_address = I2CXL_DEFAULT_ADDRESS;
	i2cxl_sonar->distance_cm = 0;
	i2cxl_sonar->distance_m  = 0;

	///< Init I2C bus
	static twi_options_t twi_opt = 
	{
		.pba_hz = 64000000,
		.speed  = 100000,
		.chip   = 1,
		.smbus  = false
	};

	twi_master_init(&AVR32_TWIM1, &twi_opt);
	dbg_print("i2cxl Sonar initialized");
}


void i2cxl_sonar_update(i2cxl_sonar_t* i2cxl_sonar)
{
	i2cxl_get_last_measure(i2cxl_sonar);
	i2cxl_send_range_command(i2cxl_sonar);
}

void i2cxl_send_range_command(i2cxl_sonar_t* i2cxl_sonar)
{
	uint8_t buff = I2CXL_RANGE_COMMAND;
	twim_write(&AVR32_TWIM1, &buff, 1, i2cxl_sonar->i2c_address, false);
}


void i2cxl_get_last_measure(i2cxl_sonar_t* i2cxl_sonar)
{
	uint8_t buf[2];
	twim_read(&AVR32_TWIM1, buf, 2, i2cxl_sonar->i2c_address, false);
	i2cxl_sonar->distance_cm = (buf[0]<<8) + buf[1];
	i2cxl_sonar->distance_m  = ((float)i2cxl_sonar->distance_cm) / 100;
}
