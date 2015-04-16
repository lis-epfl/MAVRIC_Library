/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file i2c_avr32.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief I2C peripheral driver for AVR32
 *
 ******************************************************************************/

#include "i2c_avr32.h"

extern "C"
{
	#include "print_util.h"
	#include "gpio.h"
	#include "sysclk.h"
}

i2c_avr32::i2c_avr32(i2c_avr32_conf_t config)
{
	_config = config;
}


bool i2c_avr32::init(void)
{
	switch( _config.i2c_device ) 
	{
	case AVR32_I2C0: 
		_twim = &AVR32_TWIM0;
		///< Register PDCA IRQ interrupt.
		// INTC_register_interrupt( (__int_handler) &i2c_int_handler_i2c0, AVR32_TWIM0_IRQ, AVR32_INTC_INT1);
		gpio_enable_module_pin( _config.clk_pin, 
								AVR32_TWIMS0_TWCK_0_0_FUNCTION );
		gpio_enable_module_pin( _config.sda_pin, 
								AVR32_TWIMS0_TWD_0_0_FUNCTION );

	break;
	case AVR32_I2C1:
		_twim = &AVR32_TWIM1;///< Register PDCA IRQ interrupt.
		//INTC_register_interrupt( (__int_handler) &i2c_int_handler_i2c1, AVR32_TWIM1_IRQ, AVR32_INTC_INT1);
		gpio_enable_module_pin( _config.clk_pin, 
								AVR32_TWIMS1_TWCK_0_0_FUNCTION );
		gpio_enable_module_pin( _config.sda_pin, 
								AVR32_TWIMS1_TWD_0_0_FUNCTION );
	break;
	default: ///< invalid device ID
		return false;
	}
	
	_config.twi_opt.pba_hz = sysclk_get_pba_hz();
	
	status_code_t status;
	status = twim_master_init(_twim, &_config.twi_opt);
	
	return status_code_to_bool(status);
}


bool i2c_avr32::probe(uint32_t address)
{
	status_code_t status;	
	status = twim_probe(_twim, address);
	return status_code_to_bool(status);
}


bool i2c_avr32::write(const uint8_t *buffer, uint32_t nbytes, uint32_t address)
{
	status_code_t status;
	status = twim_write(_twim, buffer, nbytes, address, _config.tenbit);
	return status_code_to_bool(status);
}


bool i2c_avr32::read(uint8_t *buffer, uint32_t nbytes, uint32_t address)
{		
	status_code_t status;
	status = twim_read(_twim, buffer, nbytes, address, _config.tenbit);
	return status_code_to_bool(status);
}


bool i2c_avr32::status_code_to_bool(status_code_t status)
{
	bool res = false;
	
	switch(status)
	{
		case STATUS_OK:
			res = true;
		break;
		case ERR_IO_ERROR:
			res = false;
		break;
		default:
			res = true;
		break;
	}

	return res;
}