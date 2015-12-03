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
 * \file main.cpp
 * 
 * \author MAV'RIC Team
 *   
 * \brief Main file
 *
 ******************************************************************************/

#include "mavrimini.hpp"
#include "central_data.hpp"
#include "mavlink_telemetry.hpp"
#include "tasks.hpp"

extern "C" 
{
	#include "print_util.h"
}

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


int main(int argc, char** argv)
{
	uint8_t sysid = 0;
	
	// -------------------------------------------------------------------------
	// Create board
	// -------------------------------------------------------------------------
	mavrimini_conf_t board_config = mavrimini_default_config(); 
	Mavrimini board(board_config);


	// -------------------------------------------------------------------------
	// Create central data
	// -------------------------------------------------------------------------
	// Create central data using simulated sensors
	Central_data cd = Central_data( sysid,
									board.imu, 
									board.sim.barometer(),
									board.sim.gps(), 
									board.sim.sonar(),
									board.uart0,
									board.spektrum_satellite,
									board.green_led,
									board.file_flash,
									board.battery,
									board.servo_0,
									board.servo_1,
									board.servo_2,
									board.servo_3 );


	// -------------------------------------------------------------------------
	// Initialisation
	// -------------------------------------------------------------------------
	bool init_success = true;

	// Board initialisation
	init_success &= board.init();

	// Init central data
	init_success &= cd.init();

	init_success &= mavlink_telemetry_add_onboard_parameters(&cd.mavlink_communication.onboard_parameters, &cd);

	// // Try to read from flash, if unsuccessful, write to flash
	if( onboard_parameters_read_parameters_from_storage(&cd.mavlink_communication.onboard_parameters) == false )
	{
		// onboard_parameters_write_parameters_to_storage(&cd.mavlink_communication.onboard_parameters);
		init_success = false; 
	}

	init_success &= mavlink_telemetry_init(&cd);

	cd.state.mav_state = MAV_STATE_STANDBY;	
	
	init_success &= tasks_create_tasks(&cd);	

	print_util_dbg_print("[MAIN] OK. Starting up.\r\n");

	// -------------------------------------------------------------------------
	// Main loop
	// -------------------------------------------------------------------------
	while (1 == 1) 
	{
		scheduler_update(&cd.scheduler);
		


		
		/**
		 * TO REMOVE (start)
		 */
		/* Toggle LEDs. */
		// gpio_toggle(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);

		// time_keeper_delay_ms(50);

		/**
		 * TO REMOVE (end)
		 */




	}

	return 0;
}
