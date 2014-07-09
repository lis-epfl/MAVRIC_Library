/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011 - 2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file compass_hmc58831l.c
*
* This file is the driver for the magnetometer HMC58831
*/

#include "compass_hmc5883l.h"
#include "twim.h"

void compass_hmc58831l_init_slow() 
{
	static twim_options_t twi_opt= 
	{
		.pba_hz	= 64000000,
		.speed	= 400000,
		.chip	= HMC5883_SLAVE_ADDRESS,
		.smbus	= false
	};
	
	static uint8_t compass_default_configuration[4] =
	{
		ConfRegA,
		(HMC_SAMPLE_AVG) << 5 | (HMC_RATE) << 2 | HMC_MODE,
		(HMC_RANGE) << 5,
		(HMC_MODE)
	};
	twim_master_init(&AVR32_TWIM0, &twi_opt);
	twim_write(&AVR32_TWIM0, (uint8_t*)&compass_default_configuration, 4, HMC5883_SLAVE_ADDRESS, false);
}


void compass_hmc58831l_update(compass_data_t *compass_outputs) 
{
	uint8_t start_address = DataRegBegin;
		
	twim_write(&AVR32_TWIM0, (uint8_t*) &start_address, 1, HMC5883_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&(compass_outputs->raw_data), 6, HMC5883_SLAVE_ADDRESS, false);
}