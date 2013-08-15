/*
 * radar_module.c
 *
 * Created: 22/04/2013 16:38:16
 *  Author: sfx
 */ 
#include "radar_module_driver.h"
//#include "i2c_driver_int.h"
#include "print_util.h"

radar_target main_target;

void init_radar_modules() {
	dbg_print("not implemented\n");

}


void read_radar() {

	dbg_print("not implemented\n");
	
}

radar_target* get_radar_main_target() {
	return &main_target;
	
}
