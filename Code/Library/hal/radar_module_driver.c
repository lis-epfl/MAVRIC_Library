/*
 * radar_module.c
 *
 * Created: 22/04/2013 16:38:16
 *  Author: sfx
 */ 
#include "radar_module_driver.h"
#include "i2c_driver_int.h"
#include "print_util.h"

mavlink_radar_tracked_target_t main_target;

void init_radar_modules() {
	static twim_options_t twi_opt= {
		.pba_hz=64000000,
		.speed = 400000,
		.chip = 1,
		.smbus=false
	};

	twim_master_init(&AVR32_TWIM1, &twi_opt);
	dbg_print("Radar modules initialised.\n");
}


void read_radar() {
	uint8_t output = 0;
//	uint8_t input [8];
	twim_write(&AVR32_TWIM1, (uint8_t*) &output, 1, 1, false);
	twim_read(&AVR32_TWIM1, (uint8_t*)&main_target, sizeof(main_target), 1, false);
	
	dbg_print_num(main_target.velocity*100.0,10);
	dbg_print_num(main_target.amplitude,10);
	dbg_print("\n");
	
}

mavlink_radar_tracked_target_t* get_radar_main_target() {
	return &main_target;
	
}
