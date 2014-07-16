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
			.up_stream = centralData.telemetry_up_stream,
			.down_stream = centralData.telemetry_down_stream,
			.sysid = MAVLINK_SYS_ID,
			.compid = 50
		},
		.message_handler_config = 
		{
			.max_msg_callback_count=20,
			.max_cmd_callback_count=20,
			.debug=true
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
	servo_pwm_failsafe((servo_output_t*)centralData.servos);
	servo_pwm_set((servo_output_t*)centralData.servos);

	// TODO change names! XXX_init()
	//imu_init((Imu_Data_t*)&(centralData.imu1));
	qfilter_init((Quat_Attitude_t*)&(centralData.imu1.attitude), (float*)(centralData.imu1.raw_scale), (float*)(centralData.imu1.raw_bias));
		
	position_estimation_init((position_estimator_t*)&(centralData.position_estimator), (pressure_data_t*)&centralData.pressure, (gps_Data_type_t*)&centralData.GPS_data);
	
	qfilter_init_quaternion((Quat_Attitude_t*)&(centralData.imu1.attitude));
		
	navigation_init();
	waypoint_handler_init(&centralData.waypoint_handler,&centralData.position_estimator,&centralData.imu1.attitude,&centralData.state_structure,&centralData.mavlink_communication);// ((waypoint_handler_t*)&centralData.waypoint_handler);
		
	neighbors_selection_init((neighbor_t*)&(centralData.neighborData), (position_estimator_t*)&(centralData.position_estimator));
	orca_init((orca_t*)&(centralData.orcaData),(neighbor_t*)&(centralData.neighborData),(position_estimator_t*)&(centralData.position_estimator),(Imu_Data_t*)&(centralData.imu1));

	// init stabilisers
	stabilisation_copter_init(	&centralData.stabilisation_copter,
								&centralData.stabiliser_stack,
								&centralData.controls,
								&centralData.run_mode,
								&centralData.imu,
								&centralData.attitude_estimation,
								&centralData.position_estimator 	);

	centralData.simulation_mode = 0;
	centralData.simulation_mode_previous = 0;

	// init simulation (should be done after position_estimator)
	simulation_init(&(centralData.sim_model), 
					&centralData.imu1.attitude,
					&(centralData.imu1),
					&centralData.position_estimator,
					&centralData.pressure,
					&centralData.GPS_data,
					&centralData.state_structure,
					HOME_LATITUDE,
					HOME_LONGITUDE,
					HOME_ALTITUDE);		


	//centralData.sim_model.localPosition = centralData.position_estimator.localPosition;

	// Init sonar
	// i2cxl_sonar_init(&centralData.i2cxl_sonar);
}

central_data_t* central_data_get_pointer_to_struct(void)
{
	return (central_data_t*)&centralData;
}