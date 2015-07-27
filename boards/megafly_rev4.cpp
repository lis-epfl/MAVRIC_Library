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

Megafly_rev4::Megafly_rev4(imu_t& imu, megafly_rev4_conf_t config):
	uart0( Serial_avr32(config.uart0_config) ), 
	uart1( Serial_avr32(config.uart1_config) ), 
	uart3( Serial_avr32(config.uart3_config) ), 
	i2c0( I2c_avr32(config.i2c0_config) ),
	i2c1( I2c_avr32(config.i2c1_config) ),
	magnetometer( Hmc5883l(i2c0, imu.raw_magneto) ),
	lsm330dlc( Lsm330dlc(i2c0, imu.raw_accelero, imu.raw_gyro) ),
	bmp085( Bmp085(i2c0) ),
	spektrum_satellite( Spektrum_satellite(uart1) ),
	imu_(imu)
{}

bool Megafly_rev4::init(void)
{
	bool init_success = true;

	Disable_global_interrupt();

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
	if( magnetometer.init() == false )
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

	return init_success;
}