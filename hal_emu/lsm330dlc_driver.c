/*
 * itg3200.c
 *
 * Created: 18/05/2012 17:57:46
 *  Author: sfx
 */ 


#include "lsm330dlc.h"
#include "print_util.h"
//#include "twim.h"

static volatile lsm_gyro_t lsm_gyro_outputs;
static volatile lsm_acc_t  lsm_acc_outputs;



void init_lsm330_acc(void) {
	
	

}

void init_lsm330_gyro(void) {

}

lsm_get_acc_config() {
}
lsm_get_gyro_config() {
}


void lsm330dlc_init(void) {

}





lsm_acc_t* lsm330dlc_driver_get_acc_data(void) {
	
	return &lsm_acc_outputs;
}


lsm_gyro_t* lsm330dlc_driver_get_gyro_data(void) {
	return &lsm_gyro_outputs;
}