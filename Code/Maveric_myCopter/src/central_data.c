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
	// Init servos
	//servo_pwm_init(central_data.servos);
	
	servos_conf_t servos_config =
	{
		.servos_count = 4,
		.types =
		{
			MOTOR_CONTROLLER,
			MOTOR_CONTROLLER,
			MOTOR_CONTROLLER,
			MOTOR_CONTROLLER
		},
	};
	servos_init( &central_data.servos, &servos_config, &central_data.mavlink_communication.mavlink_stream);
	servos_set_value_failsafe( &central_data.servos );
	pwm_servos_write_to_hardware( &central_data.servos );

	delay_ms(100);	


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
		.mav_mode = { .byte = MAV_MODE_SAFE },
		.mav_state = MAV_STATE_BOOT,
		.simulation_mode = HIL_OFF, //HIL_ON
		.autopilot_type = MAV_TYPE_QUADROTOR,
		.autopilot_name = MAV_AUTOPILOT_GENERIC,
		.sensor_present = 0b1111110000100111,
		.sensor_enabled = 0b1111110000100111,
		.sensor_health = 0b1111110000100111
	};
	state_init(	&central_data.state,
				&state_config,
				&central_data.analog_monitor,
				&central_data.mavlink_communication.mavlink_stream,
				&central_data.remote,
				&central_data.mavlink_communication.message_handler); 
	
	delay_ms(100);

	state_machine_init( &central_data.state_machine,
						&central_data.state,
						&central_data.waypoint_handler,
						&central_data.sim_model,
						&central_data.remote);
	delay_ms(100);

	// Init imu
	imu_init(	&central_data.imu,
				&central_data.mavlink_communication.mavlink_stream);
	
	delay_ms(100);

	// Init ahrs
	ahrs_init(	&central_data.ahrs,
				&central_data.mavlink_communication.mavlink_stream);

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
								&central_data.servos 	);
	
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
					&central_data.servos,
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
	
	// Init sonar
	// i2cxl_sonar_init(&central_data.i2cxl_sonar);

	// Init P^2 attitude controller
	attitude_controller_p2_conf_t attitude_controller_p2_config =
	{
		.p_gain_angle =
		{
			0.11f, 0.12f, 0.3f
		},
		.p_gain_rate =
		{
			0.08f, 0.07f, 0.2f
		}
	};
	attitude_controller_p2_init( 	&central_data.attitude_controller,
									&attitude_controller_p2_config,
									&central_data.command.attitude,
									&central_data.command.torque,
									&central_data.ahrs );

	// Init servo mixing
	servo_mix_quadcopter_diag_conf_t servo_mix_config =
	{
		.motor_front_right		= M_FRONT_RIGHT,
		.motor_front_left		= M_FRONT_LEFT,
		.motor_rear_right		= M_REAR_RIGHT,
		.motor_rear_left		= M_REAR_LEFT,
		.motor_front_right_dir	= M_FR_DIR,
		.motor_front_left_dir	= M_FL_DIR,
		.motor_rear_right_dir	= M_RR_DIR,
		.motor_rear_left_dir	= M_RL_DIR,
		.min_thrust				= MIN_THRUST,
		.max_thrust				= MAX_THRUST
	};
	servo_mix_quadcotper_diag_init( &central_data.servo_mix, 
									&servo_mix_config, 
									&central_data.command.torque, 
									&central_data.command.thrust, 
									&central_data.servos);

	// Init remote
	remote_conf_t remote_config =
	{
		.type = REMOTE_TURNIGY,
//		.mode_config =
//		{
//			.safety_channel = CHANNEL_GEAR,
//			.safety_mode = 
//			{
//				// .byte = MAV_MODE_ATTITUDE_CONTROL
//				.flags = 
//				{
//					.ARMED = ARMED_OFF,
//					.MANUAL = MANUAL_ON
//				}
//			},
//			.mode_switch_channel = CHANNEL_FLAPS,
//			.mode_switch_up = 
//			{
//				// .byte = MAV_MODE_VELOCITY_CONTROL 
//				.byte = 2 
//			},
//			.mode_switch_middle = 
//			{
//				// .byte = MAV_MODE_POSITION_HOLD 
//				.byte = 4 
//			},
//			.mode_switch_down = 
//			{
//				// .byte = MAV_MODE_GPS_NAVIGATION 
//				.byte = 8 
//			},
//			.use_custom_switch = false,
//			.custom_switch_channel = CHANNEL_AUX1,
//			.use_test_switch = false,
//			.test_switch_channel = CHANNEL_AUX2,
//		}
	};
	remote_init( 	&central_data.remote, 
					&remote_config, 
					&central_data.mavlink_communication.mavlink_stream );

	// Initialize SD/MMC driver with SPI clock (PBA).
	sd_spi_init(&central_data.sd_spi);
}

central_data_t* central_data_get_pointer_to_struct(void)
{
	return (central_data_t*)&central_data;
}