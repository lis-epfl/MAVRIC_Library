/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file central_data.c
 *
 *  Place where the central data is stored and initialized
 */


#include "central_data.h"
#include "conf_constants.h"

static central_data_t centralData;

void central_data_init()
{	
	// init mavlink
	mavlink_communication_conf_t mavlink_config = 
	{	
		.mavlink_stream_config = 
		{
			.up_stream   = centralData.telemetry_up_stream,
			.down_stream = centralData.telemetry_down_stream,
			.sysid       = MAVLINK_SYS_ID,
			.compid      = 50
		},
		.message_handler_config = 
		{
			.max_msg_callback_count = 10,
			.max_cmd_callback_count = 10,
			.debug                  = true
		},
		.onboard_parameters_config =
		{
			.max_param_count = MAX_ONBOARD_PARAM_COUNT,
			.debug           = true
		}
	};
	mavlink_communication_init(&centralData.mavlink_communication, &mavlink_config);
	
	state_init(	&centralData.state_structure,
				MAV_TYPE_QUADROTOR,
				MAV_AUTOPILOT_GENERIC,
				MAV_STATE_BOOT,
				MAV_MODE_PREFLIGHT,
				REAL_MODE,// SIMULATION_MODE
				&centralData.mavlink_communication); 
	
	// Init servos
	servo_pwm_init((servo_output_t*)centralData.servos);
	
	//imu_init((Imu_Data_t*)&(centralData.imu));
	qfilter_init((qfilter_t*)&(centralData.attitude_filter), (Imu_Data_t*)&centralData.imu, (AHRS_t*)&centralData.attitude_estimation);
		
	position_estimation_init(   &centralData.position_estimator,
								&centralData.pressure,
								&centralData.GPS_data,
								&centralData.attitude_estimation,
								&centralData.imu,
								&centralData.sim_model.localPosition,
								&centralData.waypoint_handler.waypoint_set,
								&centralData.mavlink_communication,
								HOME_LATITUDE,
								HOME_LONGITUDE,
								HOME_ALTITUDE,
								GRAVITY);
			
	navigation_init(&centralData.navigationData,
					&centralData.controls_nav,
					&centralData.attitude_estimation.qe,
					&centralData.waypoint_handler,
					&centralData.position_estimator,
					&centralData.orcaData);
					
	waypoint_handler_init(  &centralData.waypoint_handler,
							&centralData.position_estimator,
							&centralData.attitude_estimation,
							&centralData.state_structure,
							&centralData.mavlink_communication);// ((waypoint_handler_t*)&centralData.waypoint_handler);
		
	neighbors_selection_init(   &centralData.neighborData, 
								&centralData.position_estimator,
								&centralData.mavlink_communication);
	
	orca_init(  &centralData.orcaData,
				&centralData.neighborData,
				&centralData.position_estimator,
				&centralData.imu,
				&centralData.attitude_estimation);

	// init stabilisers
	stabilisation_copter_init(	&centralData.stabilisation_copter,
								&centralData.stabiliser_stack,
								&centralData.controls,
								&centralData.run_mode,
								&centralData.imu,
								&centralData.attitude_estimation,
								&centralData.position_estimator 	);

	// init simulation (should be done after position_estimator)
	simulation_init(&centralData.sim_model,
					&centralData.attitude_filter,
					&centralData.imu,
					&centralData.position_estimator,
					&centralData.pressure,
					&centralData.GPS_data,
					&centralData.state_structure,
					centralData.position_estimator.localPosition.origin.latitude,
					centralData.position_estimator.localPosition.origin.longitude,
					centralData.position_estimator.localPosition.origin.altitude,
					GRAVITY);

	// Init sonar
	// i2cxl_sonar_init(&centralData.i2cxl_sonar);
}

central_data_t* central_data_get_pointer_to_struct(void)
{
	return (central_data_t*)&centralData;
}