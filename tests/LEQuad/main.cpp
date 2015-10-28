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

#include "mavrinux.hpp"

#include "central_data.hpp"
#include "mavlink_telemetry.hpp"
#include "tasks.hpp"

extern "C" 
{
	#include "time_keeper.h"
	#include "print_util.h"
}

void initialisation(Central_data& central_data, Mavrinux& board) 
{	
	bool init_success = true;

	// Board initialisation
	init_success &= board.init();

	// Init central data
	init_success &= central_data.init();

	init_success &= mavlink_telemetry_add_onboard_parameters(&central_data.mavlink_communication.onboard_parameters, &central_data);

	// Try to read from flash, if unsuccessful, write to flash
	if( onboard_parameters_read_parameters_from_storage(&central_data.mavlink_communication.onboard_parameters) == false )
	{
		onboard_parameters_write_parameters_to_storage(&central_data.mavlink_communication.onboard_parameters);
		init_success = false; 
	}

	init_success &= mavlink_telemetry_init(&central_data);

	central_data.state.mav_state = MAV_STATE_STANDBY;	
	
	init_success &= tasks_create_tasks(&central_data);	

	print_util_dbg_print("[MAIN] OK. Starting up.\r\n");
}

#include <array>

#define S 3

int main (void)
{
	// -------------------------------------------------------------------------
	// Create board
	// -------------------------------------------------------------------------
	mavrinux_conf_t board_config = mavrinux_default_config(); 
	Mavrinux board(board_config);

	// -------------------------------------------------------------------------
	// Create central data
	// -------------------------------------------------------------------------
	// Create central data using real sensors
	Central_data cd = Central_data( board.imu, 
									board.sim.barometer(),
									board.sim.gps(), 
									board.sim.sonar(),
									board.mavlink_serial,
									board.spektrum_satellite,
									board.file_flash,
									board.battery,
									board.servos );


	initialisation(cd, board);

	Buffer_tpl<S, uint32_t> buf;
	// Buffer buf;
	for( uint32_t i = 0; i < 2*S+2; ++i )
	{
		// buf.put_lossy(i);
		buf.put_lossy(i);
		print_util_dbg_print("Put");
		print_util_dbg_print_num(i, 10);
		print_util_dbg_print(", available ");
		print_util_dbg_print_num(buf.available(), 10);		
		print_util_dbg_print(", writeable  ");
		print_util_dbg_print_num(buf.writeable(), 10);
		print_util_dbg_print("\r\n");
	}
	print_util_dbg_print("\r\n");	
	for(uint32_t i = 0; i < S; ++i)
	{
		uint32_t byte = 0;
		if( buf.get(byte) )
		{
			print_util_dbg_print("Get");
			print_util_dbg_print_num(byte, 10);
			print_util_dbg_print(", available ");
			print_util_dbg_print_num(buf.available(), 10);		
			print_util_dbg_print(", writeable  ");
			print_util_dbg_print_num(buf.writeable(), 10);
			print_util_dbg_print("\r\n");
		}
		else
		{
			print_util_dbg_print("Could not get byte\r\n");
		}
	}


	while (1 == 1) 
	{
		scheduler_update(&cd.scheduler);
	}

	return 0;
}
