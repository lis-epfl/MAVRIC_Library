/*
 * radar_module.c
 *
 * Created: 22/04/2013 16:38:16
 *  Author: sfx
 */ 
#include "radar_module_driver.h"
//#include "i2c_driver_int.h"
#include "print_util.h"

radar_target_t main_target;

void radar_module_init() {
	print_util_dbg_print("not implemented\n");

}


void radar_module_read() {

	print_util_dbg_print("not implemented\n");
	
}

radar_target_t* radar_module_get_main_target() {
	return &main_target;
	
}
