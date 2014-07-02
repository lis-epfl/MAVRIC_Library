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
* \file compass_hmc58831l.c
*
* This file is the driver for the magnetometer HMC58831
*/

#include "compass_hmc5883l.h"
#include "twim.h"

static volatile compass_data compass_outputs;		///< Declare a structure of magnetometer's data

void init_hmc5883_slow() {
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
		(HMC_SAMPLE_AVG)<<5 | (HMC_RATE) <<2 | HMC_MODE,
		(HMC_RANGE) <<5,
		(HMC_MODE)
	};
	twim_master_init(&AVR32_TWIM0, &twi_opt);
	twim_write(&AVR32_TWIM0, (uint8_t*)&compass_default_configuration, 4, HMC5883_SLAVE_ADDRESS, false);
}


compass_data* get_compass_data_slow() {
	int i;
	uint8_t start_address=DataRegBegin;
	
	twim_write(&AVR32_TWIM0, (uint8_t*) &start_address, 1, HMC5883_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&(compass_outputs.raw_data), 6, HMC5883_SLAVE_ADDRESS, false);
	
	for (i=0; i<3; i++) 
	{
		compass_outputs.axes[i] = (int16_t)(compass_outputs.raw_data[2*i]<<8)+(int16_t)(compass_outputs.raw_data[2*i+1]);
	}
	return &compass_outputs;
}