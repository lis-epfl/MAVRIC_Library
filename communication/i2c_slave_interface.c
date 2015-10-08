/*******************************************************************************
 * Copyright (c) 2009-2015, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file i2c_slave_interface.c
 * 
 * \author MAV'RIC Team
 * \author sfx
 *   
 * \brief 
 *
 ******************************************************************************/


#include "i2c_slave_interface.h"
#include "sysclk.h"


#include "twis.h"

#include "radar_module_driver.h"
#include "doppler_radar.h"


int32_t status;

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

void i2c_slave_interface_init(int32_t device_address) {
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
