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
 * \file i2c_stm32.cpp
 * 
 * \author MAV'RIC Team
 *   
 * \brief I2C peripheral driver for STM32
 *
 ******************************************************************************/

#include <libopencm3/stm32/rcc.h>

#include "i2c_stm32.hpp"

extern "C"
{
	#include "print_util.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

I2c_stm32::I2c_stm32(i2c_stm32_conf_t config)
{
	config_ = config;
}


bool I2c_stm32::init(void)
{
	// Enable I2C clock
	switch( config_.device )
	{
		case I2C_STM32_1:
			rcc_periph_clock_enable(RCC_I2C1);
		break;

		case I2C_STM32_2:
			rcc_periph_clock_enable(RCC_I2C2);
		break;

		case I2C_STM32_3:
			rcc_periph_clock_enable(RCC_I2C3);
		break;
	}

	// Setup GPIO pin for sda
	gpio_mode_setup(config_.sda_port, 
					GPIO_MODE_AF, 
					GPIO_PUPD_NONE, 
					config_.sda_pin);

	// Setup GPIO pin for clk
	gpio_mode_setup(config_.clk_port, 
					GPIO_MODE_AF, 
					GPIO_PUPD_NONE, 
					config_.clk_pin);
	
	// Setup SDA and CLK pins alternate function
	gpio_set_af(config_.sda_port, config_.sda_af, config_.sda_pin);
	gpio_set_af(config_.clk_port, config_.clk_af, config_.clk_pin);

	// Setup I2C parameters
	i2c_reset(config_.device);
	i2c_peripheral_disable(config_.device);
	i2c_set_clock_frequency(config_.device, config_.clk_freq);
	i2c_set_own_7bit_slave_address(config_.device, 0);
	i2c_peripheral_enable(config_.device);

	return true;
}


bool I2c_stm32::probe(uint32_t address)
{
	#warning I2c_stm32::probe not implemented

	return true;
}


bool I2c_stm32::write(const uint8_t *buffer, uint32_t nbytes, uint32_t address)
{
	// Wait for i2c to be ready
	while( i2c_busy(i2c) )
	{
		;
	}

	// Send start signal
	i2c_send_start(i2c);
	while(i2c_start_generated(i2c) == 0)
	{
		;
	}

	// Send address
	i2c_send_7bit_address(i2c, address, I2C_WRITE);
	while( !i2c_address_sent(i2c) ) 
	{
		if( i2c_nack_received(i2c) )
		{
			i2c_send_stop(i2c);
			return false;
		}
	}

	// Send bytes one by one
	for (uint32_t i = 0; i < nbytes; ++i)
	{
		i2c_send_data(i2c, buffer[i]);
		
		while( !(i2c_byte_transfer_finished(i2c) || i2c_nack_received(i2c)) )
		{
			;
		}
	}

	// Send stop signal
	i2c_send_stop(i2c);

	return true;
}


bool I2c_stm32::read(uint8_t *buffer, uint32_t nbytes, uint32_t address)
{		

	// while (i2c_busy(i2c));
	// i2c_send_start(i2c);
	// while (i2c_start_generated(i2c) == 0);
	// i2c_send_7bit_address(i2c, address, I2C_WRITE);
	// while (!i2c_address_sent(i2c)) {
	// 	if (i2c_nack_received(i2c)) {
	// 		i2c_send_stop(i2c);
	// 		return 0;
	// 	}
	// }
	// i2c_send_data(i2c, reg);
	// while (!(i2c_byte_transfer_finished(i2c) || i2c_nack_received(i2c)));
	// i2c_send_start(i2c);
	// while (i2c_start_generated(i2c) == 0);
	// i2c_send_7bit_address(i2c, address, I2C_READ);
	// while (!(i2c_address_sent(i2c) || i2c_nack_received(i2c)));
	// i2c_disable_ack(i2c);
	// i2c_nack_current(i2c);
	// while (!i2c_data_received(i2c));
	// uint8_t value = i2c_get_data(i2c);
	// i2c_send_stop(i2c);
	// return value;

	return true;
}




// uint8_t i2c_write_byte_reg(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t value) {

// }

// uint8_t i2c_read_byte_reg(uint32_t i2c, uint8_t addr, uint8_t reg) {

// }

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------