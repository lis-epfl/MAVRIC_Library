/* The MAV'RIC Framework
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
#include "stabilisation_copter.h"

#include "remote_controller.h"
#include "pid_control.h"
#include "streams.h"
#include "print_util.h"

#include "bmp085.h"
#include "mavlink_stream.h"
#include "coord_conventions.h"
#include "onboard_parameters.h"
#include "servo_pwm.h"

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

/**
 * \brief The central data structure
 */
typedef struct  {
	analog_monitor_t adc;										///< The analog to digital converter structure

	Imu_Data_t imu1;											///< The IMU structure
	Control_Command_t controls;									///< The control structure used for rate and attitude modes
	Control_Command_t controls_nav;								///< The control nav structure used for velocity modes
	run_mode_t run_mode;										///< The mode of the motors (MOTORS_ON, MOTORS_OFF

	Stabiliser_Stack_copter_t stabiliser_stack;					///< The stabilisation stack structure (rates, attitude, velocity, thrust)

	servo_output servos[NUMBER_OF_SERVO_OUTPUTS];				///< The array of servos (size NUMBER_OF_SERVO_OUTPUTS)
	Buffer_t xbee_in_buffer;									///< The XBEE incoming buffer
	Buffer_t wired_in_buffer;									///< The wired incoming buffer
	byte_stream_t xbee_out_stream;								///< The XBEE outgoing byte stream
	byte_stream_t xbee_in_stream;								///< The XBEE incoming byte stream
	byte_stream_t wired_out_stream;								///< The wired outgoing byte stream
	byte_stream_t wired_in_stream;								///< The wired incoming byte stream
	
	Buffer_t gps_buffer;										///< The GPS buffer
	byte_stream_t gps_stream_in;								///< The incoming GPS byte stream 
	byte_stream_t gps_stream_out;								///< The outgoing GPS byte stream 
	gps_Data_type GPS_data;										///< The GPS structure
	
	simulation_model_t sim_model;								///< The simulation model structure
	
	position_estimator_t position_estimator;					///< The position estimaton structure
	
	// aliases
	byte_stream_t *telemetry_down_stream;						///< The pointer to the downcoming telemetry byte stream
	byte_stream_t *telemetry_up_stream;							///< The pointer to the upcoming telemetry byte stream
	byte_stream_t *debug_out_stream;							///< The pointer to the outgoing debug byte stream
	byte_stream_t *debug_in_stream;								///< The pointer to the incoming debug byte stream
	
	waypoint_struct waypoint_list[MAX_WAYPOINTS];				///< The array of all waypoints (max MAX_WAYPOINTS)
	waypoint_struct current_waypoint;							///< The structure of the current waypoint
	uint16_t number_of_waypoints;								///< The total number of waypoints
	int8_t current_waypoint_count;									///< The number of the current waypoint
	
	local_coordinates_t waypoint_coordinates;					///< The coordinates of the waypoint in GPS navigation mode (MAV_MODE_AUTO_ARMED)
	local_coordinates_t waypoint_hold_coordinates;				///< The coordinates of the waypoint in position hold mode (MAV_MODE_GUIDED_ARMED)
	local_coordinates_t waypoint_critical_coordinates;			///< The coordinates of the waypoint in critical state
	float dist2wp_sqr;											///< The square of the distance to the waypoint
	
	float dist2vel_gain;										///< The gain linking the distance to the goal to the actual speed
	float cruise_speed;											///< The cruise speed in m/s
	float max_climb_rate;										///< Max climb rate in m/s
	float softZoneSize;											///< Soft zone of the velocity controller
	
	bool waypoint_set;											///< Flag to tell that a flight plan (min 1 waypoint) is active
	bool waypoint_sending;										///< Flag to tell whether waypoint are being sent
	bool waypoint_receiving;									///< Flag to tell whether waypoint are being received or not
	bool critical_landing;										///< Flag to execute critical landing (switching motors off)
	bool critical_next_state;									///< Flag to change critical state in its dedicated state machine
	
	bool collision_avoidance;									///< Flag to tell whether the collision avoidance is active or not
	bool automatic_take_off;									///< Flag to initiate the auto takeoff procedure
	bool automatic_landing;										///< Flag to initiate the auto landing procedure
	bool in_the_air;											///< Flag to tell whether the vehicle is airborne or not
	
	uint8_t mav_mode;											///< The value of the MAV mode (MAV_MODE enum in common.h)
	uint8_t mav_state;											///< The value of the MAV state (MAV_STATE enum in common.h)
	
	uint8_t mav_mode_previous;									///< The value of the MAV mode at previous time step
	uint8_t mav_state_previous;									///< The value of the MAV state at previous time step
	
	uint32_t simulation_mode;									///< The value of the simulation_mode (0: real, 1: simulation)
	uint32_t simulation_mode_previous;							///< The value of the simulation_mode at previous time step
	
	pressure_data pressure;										///< The pressure structure
	//float pressure_filtered;									///< The filtered pressure
	//float altitude_filtered;									///< The filtered altitude
	
	uint8_t number_of_neighbors;								///< The actual number of neighbors at a given time step
	float safe_size;											///< The safe size for collision avoidance
	track_neighbor_t listNeighbors[MAX_NUM_NEIGHBORS];			///< The array of neighbor structure
	
	critical_behavior_enum critical_behavior;					///< The critical behavior enum
	auto_landing_enum_t auto_landing_enum;						///< The autolanding enum

	i2cxl_sonar_t i2cxl_sonar;									///< The i2cxl sonar structure
} central_data_t;


/**
 * \brief	Initialization of the central data structure
 */
void initialise_central_data(void);

/**
 * \brief	Get a pointer to the central data
 *
 * \return	A pointer to the structure central data
*/
central_data_t* get_central_data(void);

#ifdef __cplusplus
}
#endif

#endif /* CENTRAL_DATA_H_ */