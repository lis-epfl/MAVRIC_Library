// Copyright (C) 2014  Julien Lecoeur

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.


#include "i2cxl_sonar.h"
#include "twim.h"
#include "delay.h"
#include "print_util.h"


const uint8_t I2CXL_DEFAULT_ADDRESS				= 0xE0;
const uint8_t I2CXL_RANGE_COMMAND				= 0x51;
const uint8_t I2CXL_CHANGE_ADDRESS_COMMAND_1	= 0xAA;
const uint8_t I2CXL_CHANGE_ADDRESS_COMMAND_2	= 0xA5;


void i2cxl_send_range_command(i2cxl_sonar_t* i2c_sonar);
void i2cxl_get_last_measure(i2cxl_sonar_t* i2c_sonar);


void i2cxl_sonar_init(i2cxl_sonar_t* i2cxl_sonar)
{
	// Init data_struct
	i2cxl_sonar->i2c_address = I2CXL_DEFAULT_ADDRESS;
	i2cxl_sonar->distance_cm = 0;

	// Init I2C bus
	static twi_options_t twi_opt = {.pba_hz = 64000000,
									.speed  = 100000,
									.chip   = 1,
									.smbus  = false};

	twi_master_init(&AVR32_TWIM1, &twi_opt);
	dbg_print("i2cxl Sonar initialised");
}


void i2cxl_update(i2cxl_sonar_t* i2cxl_sonar)
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
	// uint16_t distance = 0;
	twim_read(&AVR32_TWIM1, (uint8_t*)i2cxl_sonar->distance_cm, 2, i2cxl_sonar->i2c_address, false);
}
