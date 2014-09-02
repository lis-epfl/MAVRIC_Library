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
 * \file central_data.h
 *
 *  Place where the central data is stored and initialized
 */


#ifndef CENTRAL_DATA_H_
#define CENTRAL_DATA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stdbool.h"

#include "time_keeper.h"
#include "qfilter.h"
#include "imu.h"
#include "ahrs.h"
#include "stabilisation_copter.h"

#include "remote_controller.h"
#include "pid_control.h"
#include "streams.h"
#include "buffer.h"
#include "print_util.h"

#include "mavlink_stream.h"
#include "mavlink_communication.h"
#include "coord_conventions.h"
#include "onboard_parameters.h"
#include "gps_ublox.h"
#include "mavlink_waypoint_handler.h"
#include "simulation.h"
#include "bmp085.h"
#include "neighbor_selection.h"
#include "position_estimation.h"

#include "analog_monitor.h"
#include "i2cxl_sonar.h"
#include "orca.h"
#include "navigation.h"
#include "state.h"
#include "stabilisation.h"

#include "hud.h"
#include "sd_spi.h"

#include "attitude_controller_p2.h"
#include "servos.h"
#include "pwm_servos.h"
#include "servos_mix_quadcopter_diag.h"
#include "remote.h"

#include "state_machine.h"
#include "data_logging.h"

// TODO : update documentation

/**
 * \brief The central data structure
 */
typedef struct  {
	scheduler_t	scheduler;
	mavlink_communication_t mavlink_communication;
	attitude_controller_p2_t attitude_controller;
	command_t command;
	servo_mix_quadcotper_diag_t servo_mix;
	servos_t servos;
	remote_t remote;
	remote_mode_t remote_mode;

	analog_monitor_t analog_monitor;							///< The analog to digital converter structure

	imu_t imu;													///< The IMU structure
	qfilter_t attitude_filter;									///< The qfilter structure
	ahrs_t ahrs;												///< The attitude estimation structure
	control_command_t controls;									///< The control structure used for rate and attitude modes
	control_command_t controls_nav;								///< The control nav structure used for velocity modes

	stabilise_copter_t stabilisation_copter;					///< The stabilisation structure for copter

	gps_t gps;													///< The GPS structure
	
	simulation_model_t sim_model;								///< The simulation model structure
	
	position_estimator_t position_estimator;					///< The position estimaton structure
	
	// aliases
	byte_stream_t *telemetry_down_stream;						///< The pointer to the downcoming telemetry byte stream
	byte_stream_t *telemetry_up_stream;							///< The pointer to the upcoming telemetry byte stream
	byte_stream_t *debug_out_stream;							///< The pointer to the outgoing debug byte stream
	byte_stream_t *debug_in_stream;								///< The pointer to the incoming debug byte stream
	
	mavlink_waypoint_handler_t waypoint_handler;
	
	navigation_t navigation;									///< The structure to perform GPS navigation
	
	state_t state;												///< The structure with all state information
	state_machine_t state_machine;								///< The structure for the state machine
	
	barometer_t pressure;										///< The pressure structure
	
	orca_t orca;												///< The ORCA collision avoidance structure
	neighbors_t neighbors;										///< The neighbor structure
	
	hud_structure_t hud_structure;								///< The HUD structure

	i2cxl_sonar_t i2cxl_sonar;									///< The i2cxl sonar structure
	
	sd_spi_t sd_spi;											///< The sd_SPI driver structure
	
	data_logging_t data_logging;								///< The log data structure
	
} central_data_t;


/**
 * \brief	Initialization of the central data structure
 */
void central_data_init(void);


/**
 * \brief	Get a pointer to the central data
 *
 * \return	A pointer to the structure central data
*/
central_data_t* central_data_get_pointer_to_struct(void);

#ifdef __cplusplus
}
#endif

#endif /* CENTRAL_DATA_H_ */