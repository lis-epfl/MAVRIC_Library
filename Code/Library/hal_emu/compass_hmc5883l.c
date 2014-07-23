/*
 * compass_hmc5883.c
 *
 * Created: 12/03/2013 20:51:33
 *  Author: sfx
 */ 
#include "hmc5883l.h"
//#include "twim.h"

static volatile magnetometer_t compass_outputs;




void hmc5883l_init_slow() {
	print_util_dbg_print("not implemented\n");
}


magnetometer_t* compass_hmc58831l_get_data_slow() {
	return &compass_outputs;

}