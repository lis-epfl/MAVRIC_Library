/*
 * i2c_slave_interface.c
 *
 * Created: 22/04/2013 15:19:32
 *  Author: sfx
 */ 
#include "i2c_slave_interface.h"
#include "sysclk.h"


#include "twis.h"

#include "radar_module_driver.h"
#include "doppler_radar.h"


int status;

void i2c_slave_interface_handle_twis_receive(uint8_t data){
	status=data;
}

uint8_t i2c_slave_interface_handle_twis_reply() {
	uint8_t *radar_output= (uint8_t*) get_tracked_target();
	uint8_t ret=radar_output[status];
	status++;
	return ret;
}

void i2c_slave_interface_handle_twis_stop() {
	
}

void i2c_slave_interface_init(int device_address) {
	twis_options_t twis_options;
		twis_options.pba_hz =sysclk_get_pba_hz(),
		//! The baudrate of the TWI bus.
		twis_options.speed= 400000,
		//! The desired address.
		twis_options.chip=device_address;
		//! smbus mode
		twis_options.smbus=false;
		//! tenbit mode
		twis_options.tenbit=false;
	
	twis_slave_fct_t twis_functions={
		.rx=&i2c_slave_interface_handle_twis_receive,
		.tx=&i2c_slave_interface_handle_twis_reply,
		.stop=&i2c_slave_interface_handle_twis_stop
	};
	twis_slave_init(&AVR32_TWIS1, &twis_options, &twis_functions);
}	
