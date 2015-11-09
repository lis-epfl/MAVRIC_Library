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
 * \file central_data.c
 *
 * \author MAV'RIC Team
 *   
 * \brief Place where the central data is stored and initialized
 *
 ******************************************************************************/


#include "central_data.hpp"
#include "stabilisation_copter_default_config.hpp"
#include "data_logging_default_config.hpp"
#include "mavlink_communication_default_config.hpp"

#include "position_estimation_default_config.hpp"
#include "remote_default_config.hpp"
#include "state_default_config.hpp"
#include "manual_control_default_config.hpp"
#include "toggle_logging.hpp"

extern "C" 
{
	#include "time_keeper.h"
	#include "navigation_default_config.h"
	#include "qfilter_default_config.h"
	#include "scheduler_default_config.h"
	#include "servos_mix_quadcopter_diag_default_config.h"

	#include "print_util.h"
}


Central_data::Central_data(Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink, Satellite& satellite, File& file_flash, File& file_log, Battery& battery, servos_t& servos):
	imu( imu ),
	barometer( barometer ),
	gps( gps ),
	sonar( sonar ),
	serial_mavlink( serial_mavlink ),
	satellite( satellite ),
	file_flash( file_flash ),
	file_log ( file_log),
	battery( battery ),
	servos( servos )
{}


bool Central_data::init(void)
{
	bool init_success = true;
	bool ret;

	print_util_dbg_sep('%');
	time_keeper_delay_ms(100); 
	print_util_dbg_sep('-');
	time_keeper_delay_ms(100); 
	print_util_dbg_print("[CENTRAL_DATA] ...\r\n");
	time_keeper_delay_ms(100); 
	print_util_dbg_sep('-');


	// -------------------------------------------------------------------------
	// Init main sheduler
	// -------------------------------------------------------------------------
	ret = scheduler_init(&scheduler, scheduler_default_config());
	print_util_dbg_init_msg("[SCHEDULER]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 


	// -------------------------------------------------------------------------
	// Init mavlink communication
	// -------------------------------------------------------------------------
	mavlink_communication_conf_t mavlink_communication_config = mavlink_communication_default_config();
	mavlink_communication_config.mavlink_stream_config.sysid = 1;
	mavlink_communication_config.message_handler_config.debug = true;
	mavlink_communication_config.onboard_parameters_config.debug = true;
	mavlink_communication_config.mavlink_stream_config.debug = true;
	ret = mavlink_communication_init(	&mavlink_communication, 
										mavlink_communication_config, 
										&serial_mavlink,
										&state,
										&file_flash );
	print_util_dbg_init_msg("[MAVLINK]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 


	// -------------------------------------------------------------------------
	// Init state structure
	// -------------------------------------------------------------------------
	ret = state_init(	&state,
						state_default_config(),
						&battery); 
	print_util_dbg_init_msg("[STATE]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 


	// -------------------------------------------------------------------------
	//Init state_machine	
	// -------------------------------------------------------------------------
	ret = state_machine_init( 	&state_machine,
								&state,
								&gps,
								&manual_control);
	print_util_dbg_init_msg("[STATE MACHINE]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 


	// -------------------------------------------------------------------------
	// Init ahrs
	// -------------------------------------------------------------------------
	ret = ahrs_init(&ahrs);
	print_util_dbg_init_msg("[AHRS]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 
	

	// -------------------------------------------------------------------------
	// Init qfilter
	// -------------------------------------------------------------------------
	ret = qfilter_init( &attitude_filter,
						qfilter_default_config(),
						&imu,
						&ahrs);
	print_util_dbg_init_msg("[QFILTER]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 
	

	// -------------------------------------------------------------------------
	// Init position_estimation_init
	// -------------------------------------------------------------------------
	ret = position_estimation_init( &position_estimation,
									position_estimation_default_config(),
									&state,
									&barometer,
									&sonar,
									&gps,
									&ahrs);
	print_util_dbg_init_msg("[POS EST]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 


	// -------------------------------------------------------------------------
	// Init navigation
	// -------------------------------------------------------------------------
	ret = navigation_init(	&navigation,
							navigation_default_config(),
							&controls_nav,
							&ahrs.qe,
							&position_estimation,
							&state,
							&mavlink_communication);/*,
							&sonar_i2cxl);*/
	print_util_dbg_init_msg("[NAV]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 


	// -------------------------------------------------------------------------
	// Init waypoint handler
	// -------------------------------------------------------------------------
	ret = waypoint_handler_init(	&waypoint_handler,
									&position_estimation,
									&navigation,
									&ahrs,
									&state,
									&manual_control,
									&mavlink_communication,
									&mavlink_communication.mavlink_stream);
	waypoint_handler_init_homing_waypoint(&waypoint_handler);
	waypoint_handler_nav_plan_init(&waypoint_handler);
	print_util_dbg_init_msg("[WAYPOINT]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 

	
	// -------------------------------------------------------------------------
	// Init stabilisers
	// -------------------------------------------------------------------------
	ret = stabilisation_copter_init(	&stabilisation_copter,
										stabilisation_copter_default_config(),
										&controls,
										&ahrs,
										&position_estimation,
										&command.torque,
										&command.thrust);
	print_util_dbg_init_msg("[STABILISATION COPTER]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 


	// -------------------------------------------------------------------------
	// Init controls
	// -------------------------------------------------------------------------
	ret = stabilisation_init( &controls);
	print_util_dbg_init_msg("[CONTROLS]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 
	
	
	// -------------------------------------------------------------------------
	// Init hud	
	// -------------------------------------------------------------------------
	ret = hud_telemetry_init(	&hud_structure, 
								&position_estimation,
								&controls,
								&ahrs);
	print_util_dbg_init_msg("[HUD]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 
	

	// -------------------------------------------------------------------------
	// Init servo mixing
	// -------------------------------------------------------------------------
	ret = servos_mix_quadcotper_diag_init( 	&servo_mix,
											servos_mix_quadcopter_diag_default_config(),
											&command.torque,
											&command.thrust,
											&servos);
	print_util_dbg_init_msg("[SERVOS MIX]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 


	// -------------------------------------------------------------------------
	// Init manual control
	// -------------------------------------------------------------------------
	ret = manual_control_init(	&manual_control,
								&satellite,
								manual_control_default_config(),
								remote_default_config());
	print_util_dbg_init_msg("[MANUAL CTRL]", ret);
	init_success &= ret;
	time_keeper_delay_ms(100); 

	// -------------------------------------------------------------------------
	// Init data logging
	// -------------------------------------------------------------------------
	//TODO: not working here
	ret = toggle_logging_init( &toggle_logging,
								data_logging_default_config(),
								&state);



	ret = data_logging_create_new_log_file(	&data_logging,
											"Log_file",
											&file_log,
											true,
											&toggle_logging,
											mavlink_communication.mavlink_stream.sysid);
	print_util_dbg_init_msg("[DATA LOGGING]", ret);
	init_success &= ret;

	print_util_dbg_sep('-');
	time_keeper_delay_ms(100); 
	print_util_dbg_init_msg("[CENTRAL_DATA]", init_success);
	time_keeper_delay_ms(100); 
	print_util_dbg_sep('-');
	time_keeper_delay_ms(100);
	
	return init_success;
}
