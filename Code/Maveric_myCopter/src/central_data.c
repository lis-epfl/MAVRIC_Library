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
	
	simulation_config_t vehicle_model_parameters= {
		.rotor_lpf			 =  0.1f, 					///< Low pass filter constant (adjusted for time) to express rotor inertia/lag. 1.0=no inertia, 0.0=infinite inertia
		.rotor_rpm_gain		 =  4000.0f,					///< The gain linking the rotor command to rpm
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
	
	mavlink_communication_init(&centralData.mavlink_communication, &mavlink_config);
	
	state_init(	&centralData.state_structure,
				MAV_TYPE_QUADROTOR,
				MAV_AUTOPILOT_GENERIC,
				MAV_STATE_BOOT,
				MAV_MODE_PREFLIGHT,
				REAL_MODE, // SIMULATION_MODE
				&centralData.mavlink_communication.message_handler); 
	
	imu_init(&(centralData.imu));
	
	// Init servos
	servo_pwm_init(centralData.servos);
	
	qfilter_init(   &(centralData.attitude_filter), 
					&centralData.imu, 
					&centralData.attitude_estimation);
	
	position_estimation_init(   &centralData.position_estimator,
								&centralData.pressure,
								&centralData.GPS_data,
								&centralData.attitude_estimation,
								&centralData.imu,
								&centralData.sim_model.localPosition,
								&centralData.waypoint_handler.waypoint_set,
								&centralData.mavlink_communication.message_handler,
								HOME_LATITUDE,
								HOME_LONGITUDE,
								HOME_ALTITUDE,
								GRAVITY				);
	
	navigation_init(&centralData.navigationData,
					&centralData.controls_nav,
					&centralData.attitude_estimation.qe,
					&centralData.waypoint_handler,
					&centralData.position_estimator,
					&centralData.orcaData,
					&centralData.state_structure);
	
	waypoint_handler_init(  &centralData.waypoint_handler,
							&centralData.position_estimator,
							&centralData.attitude_estimation,
							&centralData.state_structure,
							&centralData.mavlink_communication);
	
	neighbors_selection_init(   &centralData.neighborData, 
								&centralData.position_estimator,
								&centralData.mavlink_communication.message_handler);
	
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
								&centralData.position_estimator,
								centralData.servos 	);
	
	// init simulation (should be done after position_estimator)
	simulation_init(&centralData.sim_model,
					&vehicle_model_parameters,
					&centralData.attitude_estimation,
					&centralData.imu,
					&centralData.position_estimator,
					&centralData.pressure,
					&centralData.GPS_data,
					&centralData.state_structure,
					centralData.servos);

	// Init sonar
	// i2cxl_sonar_init(&centralData.i2cxl_sonar);
}

central_data_t* central_data_get_pointer_to_struct(void)
{
	return (central_data_t*)&centralData;
}