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

static central_data_t central_data;

void central_data_init()
{	
	// Init ahrs
	servo_pwm_init(central_data.servos);
	
	// Init main sheduler
	scheduler_conf_t scheduler_config =
	{
		.max_task_count = 10,
		.schedule_strategy = ROUND_ROBIN,
		.debug = true
	};
	scheduler_init(&central_data.scheduler, &scheduler_config, &central_data.mavlink_communication.mavlink_stream);
	
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
			.rx_stream   = central_data.telemetry_up_stream,
			.tx_stream   = central_data.telemetry_down_stream,
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
	mavlink_communication_init(&central_data.mavlink_communication, &mavlink_config);
	
	delay_ms(100); 

	// Init state structure
	state_t state_config =
	{
		.mav_mode = MAV_MODE_SAFE,
		.mav_state = MAV_STATE_BOOT,
		.simulation_mode = REAL_MODE , //SIMULATION_MODE
		.autopilot_type = MAV_TYPE_QUADROTOR,
		.autopilot_name = MAV_AUTOPILOT_GENERIC,
		.sensor_present = 0b1111110000100111,
		.sensor_enabled = 0b1111110000100111,
		.sensor_health = 0b1111110000100111,
		.remote_active = false
	};
	state_init(	&central_data.state,
				&state_config,
				&central_data.analog_monitor,
				&central_data.mavlink_communication.mavlink_stream,
				&central_data.mavlink_communication.message_handler); 
	
	delay_ms(100);

	state_machine_init( &central_data.state_machine,
						&central_data.state,
						&central_data.waypoint_handler,
						&central_data.sim_model);
	delay_ms(100);

	// Init imu
	imu_init(	&central_data.imu,
				&central_data.mavlink_communication.mavlink_stream);
	
	delay_ms(100);

	// Init ahrs
	ahrs_init(	&central_data.ahrs,
				&central_data.mavlink_communication.mavlink_stream);

	delay_ms(100);

	
	
	delay_ms(100);
	
	// Init qfilter
	qfilter_init(   &(central_data.attitude_filter), 
					&central_data.imu, 
					&central_data.ahrs);
	
	delay_ms(100);
	
	// Init position_estimation_init
	position_estimation_init(   &central_data.position_estimator,
								&central_data.state,
								&central_data.pressure,
								&central_data.gps,
								&central_data.ahrs,
								&central_data.imu,
								&central_data.mavlink_communication.mavlink_stream,
								&central_data.state.nav_plan_active,
								&central_data.mavlink_communication.message_handler,
								HOME_LATITUDE,
								HOME_LONGITUDE,
								HOME_ALTITUDE,
								GRAVITY				);
	
	delay_ms(100);

	// Init navigation
	navigation_init(&central_data.navigation,
					&central_data.controls_nav,
					&central_data.ahrs.qe,
					&central_data.waypoint_handler,
					&central_data.position_estimator,
					&central_data.orca,
					&central_data.state,
					&central_data.mavlink_communication.mavlink_stream);
	
	delay_ms(100);

	// Init waypont handler
	waypoint_handler_init(  &central_data.waypoint_handler,
							&central_data.position_estimator,
							&central_data.ahrs,
							&central_data.state,
							&central_data.mavlink_communication,
							&central_data.mavlink_communication.mavlink_stream);
	waypoint_handler_init_homing_waypoint(&central_data.waypoint_handler);
	waypoint_handler_nav_plan_init(&central_data.waypoint_handler);
	
	delay_ms(100);

	// Init neighbor selection
	neighbors_selection_init(   &central_data.neighbors, 
								&central_data.position_estimator,
								&central_data.mavlink_communication.message_handler,
								&central_data.mavlink_communication.mavlink_stream);
	
	delay_ms(100);

	// Init orca
	orca_init(  &central_data.orca,
				&central_data.neighbors,
				&central_data.position_estimator,
				&central_data.imu,
				&central_data.ahrs);
	
	delay_ms(100);

	// Init stabilisers
	stabilisation_copter_init(	&central_data.stabilisation_copter,
								&central_data.stabiliser_stack,
								&central_data.controls,
								&central_data.imu,
								&central_data.ahrs,
								&central_data.position_estimator,
								central_data.servos);
	
	delay_ms(100);

	stabilisation_init( &central_data.stabilisation_copter.stabiliser_stack->rate_stabiliser, 
						&central_data.controls,
						&central_data.mavlink_communication.mavlink_stream);
	
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
	simulation_init(&central_data.sim_model,
					&vehicle_model_parameters,
					&central_data.ahrs,
					&central_data.imu,
					&central_data.position_estimator,
					&central_data.pressure,
					&central_data.gps,
					&central_data.state,
					central_data.servos,
					&central_data.state.nav_plan_active,
					&central_data.mavlink_communication.message_handler,
					&central_data.mavlink_communication.mavlink_stream);

	delay_ms(100);//add delay to be able to print on console init message for the following module
	
	// Init hud	
	hud_init(	&central_data.hud_structure, 
				&central_data.position_estimator, 
				&central_data.controls, 
				&central_data.ahrs,
				&central_data.mavlink_communication.mavlink_stream);
	
	delay_ms(100);
	
	joystick_parsing_init(	&central_data.joystick_parsing,
								&central_data.controls_joystick,
								&central_data.mavlink_communication);
	delay_ms(100);
	
	// Init sonar
	// i2cxl_sonar_init(&central_data.i2cxl_sonar);
	// delay_ms(100);
	
	data_logging_conf_t data_logging_conf = 
	{
		.debug = true,
		.max_data_logging_count = MAX_DATA_LOGGING_COUNT,
		.log_data = 0 // 1: log data, 0: no log data
	};
	
	data_logging_init(  &central_data.data_logging,
						&data_logging_conf);
}

central_data_t* central_data_get_pointer_to_struct(void)
{
	return (central_data_t*)&central_data;
}