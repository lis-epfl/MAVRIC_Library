/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file 	spi_avr32.cpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Implementation of spi peripheral for avr32
 *
 ******************************************************************************/


#include "spi_avr32.hpp"

extern "C"
{
	#include "gpio.h"
	#include "sysclk.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Spi_avr32::Spi_avr32(spi_avr32_conf_t config)
{
	config_			= config;
}


bool Spi_avr32::init(void)
{	
	// Init gpios
	//MOSI
	gpio_enable_module_pin( config_.mosi_pin_map.pin,
								config_.mosi_pin_map.function );
	//MISO
	gpio_enable_module_pin( config_.miso_pin_map.pin,
								config_.miso_pin_map.function );
	//SCK
	gpio_enable_module_pin( config_.sck_pin_map.pin,
								config_.sck_pin_map.function );
	//SS
	gpio_enable_module_pin( config_.ss_pin_map.pin,
								config_.ss_pin_map.function );

	// Init pointer to device, and Register interrupt
	switch( config_.spi_device )
	{
		case AVR32_SPI_0:
			spi_ = &AVR32_SPI0;	// Assign I/Os to SPI.
		break;
		case AVR32_SPI_1:
			spi_ = &AVR32_SPI1;	// Assign I/Os to SPI.
		break;
		default:
		break;
	}

	// Initialize as master.
	spi_initMaster((spi_), &config_.options);

	// Set selection mode: variable_ps, pcs_decode, delay.
	spi_selectionMode((spi_), 0, 0, 0);

	//Set how we're talking to the chip. (Bits!  et al)
	spi_setupChipReg((spi_), &config_.options, sysclk_get_pba_hz()); //very important!

	// Enable SPI.
	spi_enable(spi_);
	
	// Configure Chip Select even if not used.
	spi_selectChip(spi_, 0);		

	return true;
}


bool Spi_avr32::write(const uint8_t* bytes, const uint32_t size)
{
	bool ret = false;

	// // Queue byte
	for (uint32_t i = 0; i < size; ++i)
	{
		spi_put(spi_, bytes[i]);
	}

	return ret;
}


bool Spi_avr32::read(uint16_t command, uint8_t* bytes, const uint32_t size)
{
	bool ret = false;

	//CS ON
	gpio_clr_gpio_pin(config_.ss_pin_map.pin);
	
	for (uint8_t i = 0; i < size; ++i)
	{
		//wait until
		while (!spi_is_tx_ready(spi_));
		
		spi_put(spi_, command);
		
		//wait until
		while(!spi_is_rx_full(spi_));
		
		bytes[i] = spi_get(spi_);
	}
	
	//CS OFF
	gpio_set_gpio_pin(config_.ss_pin_map.pin);
	
	return ret;
}
