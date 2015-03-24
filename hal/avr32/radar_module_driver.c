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
 * \file radar_module_driver.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief The radar module driver
 * 
 ******************************************************************************/
 
 
#include "radar_module_driver.h"
#include "i2c_driver_int.h"
#include "print_util.h"

void radar_module_init() 
{
	static twim_options_t twi_opt= 
	{
		.pba_hz = 64000000,
		.speed = 400000,
		.chip = 1,
		.smbus = false
	};

	twim_master_init(&AVR32_TWIM1, &twi_opt);
	print_util_dbg_print("Radar modules initialised.\r\n");;
}


void radar_module_read(radar_target_t* main_target) 
{
	uint8_t output = 0;
	//uint8_t input [8];
	twim_write(&AVR32_TWIM1, (uint8_t*) &output, 1, 1, false);
	twim_read(&AVR32_TWIM1, (uint8_t*)main_target, sizeof(main_target), 1, false);
	
	print_util_dbg_print_num(main_target->velocity * 100.0f,10);
	print_util_dbg_print_num(main_target->amplitude,10);
	print_util_dbg_print("\r\n");;
}