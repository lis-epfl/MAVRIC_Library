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
 * \file 	megafly_rev4.cpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Autopilot board based on AVR32
 *
 ******************************************************************************/

#include "megafly_rev4.hpp"

extern "C"
{
	#include "print_util.h"
	#include "time_keeper.h"
}


static Serial_usb_avr32* p_uart_usb;
uint8_t serial2stream( stream_data_t data, uint8_t byte )
{
	p_uart_usb->write(&byte);
	return 0;
}

Megafly_rev4::Megafly_rev4(megafly_rev4_conf_t config):
	dsm_receiver_pin( Gpio_avr32(config.dsm_receiver_pin_config) ),
	dsm_power_pin( Gpio_avr32(config.dsm_power_pin_config) ),
	uart0( Serial_avr32(config.uart0_config) ), 
	uart1( Serial_avr32(config.uart1_config) ), 
	uart3( Serial_avr32(config.uart3_config) ), 
	uart_usb( Serial_usb_avr32(config.uart_usb_config) ), 
	i2c0( I2c_avr32(config.i2c0_config) ),
	i2c1( I2c_avr32(config.i2c1_config) ),
	hmc5883l( Hmc5883l(i2c0) ),
	lsm330dlc( Lsm330dlc(i2c0) ),
	bmp085( Bmp085(i2c0) ),
	spektrum_satellite( Spektrum_satellite(uart1, dsm_receiver_pin, dsm_power_pin) ),
	imu( Imu(lsm330dlc, lsm330dlc, hmc5883l) ),
	file_flash( File_flash_avr32("flash.bin") ),
	gps_ublox( Gps_ublox(uart3) ),
	sonar_i2cxl( Sonar_i2cxl(i2c1) )
{}


bool Megafly_rev4::init(void)
{
	bool init_success = true;

	Disable_global_interrupt();

	// Init UART3
	if( uart_usb.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[UART USB] INIT ERROR\r\n");
	}

	// Init GPIO dsm receiver
	if( dsm_receiver_pin.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[DSM RX PIN] INIT ERROR\r\n");		
	}

	// Init GPIO dsm power
	if( dsm_power_pin.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[DSM VCC PIN] INIT ERROR\r\n");		
	}

	// Init UART0
	if( uart0.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[UART0] INIT ERROR\r\n");
	}

	// Init UART1
	if( uart1.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[UART1] INIT ERROR\r\n");
	}

	// Init UART3
	if( uart3.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[UART3] INIT ERROR\r\n");
	}
	
	// Init I2C0
	if( i2c0.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[I2C0] INIT ERROR\r\n");
	}

	// Init I2C1
	if( i2c1.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[I2C1] INIT ERROR\r\n");
	}
	
	Enable_global_interrupt();
	
	// Init magnetometer
	if( hmc5883l.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[HMC] INIT ERROR\r\n");
	}

	// Init gyro and accelero
	if( lsm330dlc.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[LSM330] INIT ERROR\r\n");
	}
			
	// Init barometer
	if( bmp085.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[BMP085] INIT ERROR\r\n");
	}

	// Init spektrum_satelitte
	if( spektrum_satellite.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[SAT] INIT ERROR\r\n");
	}

	// Init sonar
	if( sonar_i2cxl.init() == false )
	{
		init_success = false;
		print_util_dbg_print("[SONAR] INIT ERROR\r\n");		
	}		


	// -------------------------------------------------------------------------
	// Init stream for USB debug stream TODO: remove
	p_uart_usb = &uart_usb;
	dbg_stream_.get = NULL;
	dbg_stream_.put = &serial2stream;
	dbg_stream_.flush = NULL;
	dbg_stream_.buffer_empty = NULL;
	dbg_stream_.data = NULL;
	print_util_dbg_print_init(&dbg_stream_);
	// -------------------------------------------------------------------------

	return init_success;
}