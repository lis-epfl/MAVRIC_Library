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
#include "print_util.h"

void compass_hmc58831l_init_slow() 
{
	if(twim_probe(&AVR32_TWIM0, HMC5883_SLAVE_ADDRESS) == STATUS_OK)
	{
		print_util_dbg_print("HMC5883 compass sensor found (0x1E) \r");
	}
	else
	{
		print_util_dbg_print("HMC5883 compass sensor not responding (0x1E) \r");
		return;
	}
	
	static uint8_t compass_default_configuration[4] =
	{
		ConfRegA,
		(HMC_SAMPLE_AVG) << 5 | (HMC_RATE) << 2 | HMC_MODE,
		(HMC_RANGE) << 5,
		(HMC_MODE)
	};
	
	twim_write(&AVR32_TWIM0, (uint8_t*)&compass_default_configuration, 4, HMC5883_SLAVE_ADDRESS, false);
}


void compass_hmc58831l_update(compass_data_t *compass_outputs) 
{
	uint8_t start_address = DataRegBegin;
	uint16_t data[3];
		
	twim_write(&AVR32_TWIM0, (uint8_t*) &start_address, 1, HMC5883_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&(data), 6, HMC5883_SLAVE_ADDRESS, false);
	
	compass_outputs->data[0] = data[0];
	compass_outputs->data[1] = data[1];
	compass_outputs->data[2] = data[2];
}