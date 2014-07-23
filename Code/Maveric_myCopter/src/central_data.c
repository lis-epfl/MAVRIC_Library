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
#include "delay.h"

static central_data_t centralData;

void central_data_init()
{	
	// Init main sheduler
	scheduler_conf_t scheduler_config =
	{
		.max_task_count = 10,
		.schedule_strategy = ROUND_ROBIN,
		.debug = true
	};
	scheduler_init(&centralData.scheduler, &scheduler_config, &centralData.mavlink_communication.mavlink_stream);
	
	delay_ms(100); 

	// Init mavlink communication
	mavlink_communication_conf_t mavlink_config = 
	{	
		.scheduler_config =
		{
			.max_task_count = 30,
			.schedule_strategy = ROUND_ROBIN,
			.debug = true
		},
		.mavlink_stream_config = 
		{
			.rx_stream   = centralData.telemetry_up_stream,
			.tx_stream   = centralData.telemetry_down_stream,
			.sysid       = MAVLINK_SYS_ID,
			.compid      = 50,
			.use_dma     = false
		},
		.message_handler_config = 
		{
			.max_msg_callback_count = 20,
			.max_cmd_callback_count = 20,
			.debug                  = true
		},
		.onboard_parameters_config =
		{
			.max_param_count = MAX_ONBOARD_PARAM_COUNT,
			.debug           = true
		}
	};
	mavlink_communication_init(&centralData.mavlink_communication, &mavlink_config);
	
	delay_ms(100); 

	// Init state structure
	state_structure_t state_config =
	{
		.mav_mode = MAV_MODE_SAFE,
		.mav_state = MAV_STATE_BOOT,
		.simulation_mode = SIMULATION_MODE, //REAL_MODE
		.autopilot_type = MAV_TYPE_QUADROTOR,
		.autopilot_name = MAV_AUTOPILOT_GENERIC,
		.sensor_present = 0b1111110000100111,
		.sensor_enabled = 0b1111110000100111,
		.sensor_health = 0b1111110000100111
	};
	state_init(	&centralData.state_structure,
				&state_config,
				&centralData.adc,
				&centralData.mavlink_communication.mavlink_stream,
				&centralData.mavlink_communication.message_handler); 
	
	delay_ms(100);

	// Init imu
	imu_init(	&centralData.imu,
				&centralData.mavlink_communication.mavlink_stream);
	
	delay_ms(100);

	// Init ahrs
	ahrs_init(	&centralData.ahrs,
				&centralData.mavlink_communication.mavlink_stream);

	delay_ms(100);

	// Init ahrs
	servo_pwm_init(centralData.servos);
	
	delay_ms(100);
	
	// Init qfilter
	qfilter_init(   &(centralData.attitude_filter), 
					&centralData.imu, 
					&centralData.ahrs,
					&centralData.mavlink_communication.mavlink_stream);
	
	delay_ms(100);
	
	// Init position_estimation_init
	position_estimation_init(   &centralData.position_estimator,
								&centralData.pressure,
								&centralData.GPS_data,
								&centralData.ahrs,
								&centralData.imu,
								&centralData.mavlink_communication.mavlink_stream,
								&centralData.waypoint_handler.waypoint_set,
								&centralData.mavlink_communication.message_handler,
								HOME_LATITUDE,
								HOME_LONGITUDE,
								HOME_ALTITUDE,
								GRAVITY				);
	
	delay_ms(100);

	// Init navigation
	navigation_init(&centralData.navigationData,
					&centralData.controls_nav,
					&centralData.ahrs.qe,
					&centralData.waypoint_handler,
					&centralData.position_estimator,
					&centralData.orcaData,
					&centralData.state_structure);
	
	delay_ms(100);

	// Init waypont handler
	waypoint_handler_init(  &centralData.waypoint_handler,
							&centralData.position_estimator,
							&centralData.ahrs,
							&centralData.state_structure,
							&centralData.mavlink_communication);
	waypoint_handler_init_homing_waypoint(&centralData.waypoint_handler);
	waypoint_handler_waypoint_init(&centralData.waypoint_handler);
	
	delay_ms(100);

	// Init neighbor selection
	neighbors_selection_init(   &centralData.neighborData, 
								&centralData.position_estimator,
								&centralData.mavlink_communication.message_handler);
	
	delay_ms(100);

	// Init orca
	orca_init(  &centralData.orcaData,
				&centralData.neighborData,
				&centralData.position_estimator,
				&centralData.imu,
				&centralData.ahrs);
	
	delay_ms(100);

	// Init stabilisers
	stabilisation_copter_init(	&centralData.stabilisation_copter,
								&centralData.stabiliser_stack,
								&centralData.controls,
								&centralData.imu,
								&centralData.ahrs,
								&centralData.position_estimator,
								centralData.servos 	);
	
	delay_ms(100);

	// Init simulation (should be done after position_estimator)
	simulation_config_t vehicle_model_parameters= 
	{
		.rotor_lpf			 =  0.1f, 					///< Low pass filter constant (adjusted for time) to express rotor inertia/lag. 1.0=no inertia, 0.0=infinite inertia
		.rotor_rpm_gain		 =  4000.0f,				///< The gain linking the rotor command to rpm
		.rotor_rpm_offset	 =  -1.0f,					///< Offset to convert servo commands to rpm (servo command that corresponds to zero rpm)
		.rotor_cd			 =  0.03f,					///< Coefficient of drag of rotor blade
		.rotor_cl			 =  1.0f,					///< Coefficient of lift of rotor blade
		.rotor_diameter      =  0.14f,					///< Mean "effective" rotor diameter
		.rotor_foil_area	 =  0.18f * 0.015f,			///< Area of the propeller blades in m^2
		.rotor_pitch         =  0.15f,					///< Rotor pitch in m/revolution (7x6" roughly 0.15m)
		.total_mass			 =  0.35f,					///< Vehicle mass in kg
		.vehicle_drag        =  0.01f,					///< Vehicle drag coefficient * vehicle area
		.roll_pitch_momentum =  0.1f * 0.17f / 1.4142f,	///< Angular momentum constants (assumed to be independent) (in kg/m^2)
		.yaw_momentum		 =  0.1f * 0.17f ,			///< Approximate motor arm mass * rotor arm length
		.rotor_momentum      =  0.005f * 0.03f,			///< Rotor inertia  (5g off center mass * rotor radius)
		.rotor_arm_length	 =  0.17f,					///< Distance between CoG and motor (in meter)
		.wind_x				 =  0.0f,					///< Wind in x axis, global frame
		.wind_y				 =  0.0f,					///< Wind in y axis, global frame
		.home_coordinates[X] =  HOME_LATITUDE,			///< Latitude coordinate of the home position
		.home_coordinates[Y] =  HOME_LONGITUDE,			///< Longitude coordinate of the home position
		.home_coordinates[Z] =  HOME_ALTITUDE,			///< Altitude coordinate of the home position
		.sim_gravity		 =  GRAVITY					///< Simulation gravity
	};
	simulation_init(&centralData.sim_model,
					&vehicle_model_parameters,
					&centralData.ahrs,
					&centralData.imu,
					&centralData.position_estimator,
					&centralData.pressure,
					&centralData.GPS_data,
					&centralData.state_structure,
					centralData.servos,
					&centralData.waypoint_handler.waypoint_set,
					&centralData.mavlink_communication.message_handler,
					&centralData.mavlink_communication.mavlink_stream);

	delay_ms(100);//add delay to be able to print on console init message for the following module
	
	// Init hud	
	hud_init(	&centralData.hud_structure, 
				&centralData.position_estimator, 
				&centralData.controls, 
				&centralData.ahrs,
				&centralData.mavlink_communication.mavlink_stream);
	
	delay_ms(100);
	
	// Init sonar
	// i2cxl_sonar_init(&centralData.i2cxl_sonar);
}

central_data_t* central_data_get_pointer_to_struct(void)
{
	return (central_data_t*)&centralData;
}