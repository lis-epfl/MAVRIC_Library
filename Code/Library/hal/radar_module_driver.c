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
 * \file radar_module_driver.c
 * 
 * The radar module driver
 */
 
 
#include "radar_module_driver.h"
#include "i2c_driver_int.h"
#include "print_util.h"

///< Declare an object containing the mavlink radar tracked target structure
mavlink_radar_tracked_target_t main_target;

void radar_module_init() 
{
	static twim_options_t twi_opt= 
	{
		.pba_hz = 64000000,
		.speed = 400000,
		.chip = 1,
		.smbus = false
	};

	twim_master_init(&AVR32_TWIM1, &twi_opt);
	print_util_dbg_print("Radar modules initialised.\r\n");;
}


void radar_module_read() 
{
	uint8_t output = 0;
	//uint8_t input [8];
	twim_write(&AVR32_TWIM1, (uint8_t*) &output, 1, 1, false);
	twim_read(&AVR32_TWIM1, (uint8_t*)&main_target, sizeof(main_target), 1, false);
	
	print_util_dbg_print_num(main_target.velocity * 100.0f,10);
	print_util_dbg_print_num(main_target.amplitude,10);
	print_util_dbg_print("\r\n");;
}

mavlink_radar_tracked_target_t* radar_module_get_main_target() 
{
	return &main_target;	
}
