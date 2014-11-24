/*
 * central_data.h
 *
 * Created: 16/09/2013 12:18:00
 *  Author: sfx
 */ 


#ifndef CENTRAL_DATA_H_
#define CENTRAL_DATA_H_

#include <stdbool.h>
#include <stdint.h>

#include "stdbool.h"

#include "time_keeper.h"
//#include "i2c_driver_int.h"
#include "qfilter.h"
#include "imu.h"

// #include "stabilisation.h"
#include "stabilisation_copter.h"
// #include "stabilisation_hybrid.h"

#include "pid_controller.h"
#include "streams.h"
//#include "uart_int.h"
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
#include "conf_sim_model.h"
#include "position_estimation.h"

static const servo_output_t servo_failsafe[NUMBER_OF_SERVO_OUTPUTS]={{.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}};

enum CRITICAL_BEHAVIOR_ENUM{
	CLIMB_TO_SAFE_ALT = 1,
	FLY_TO_HOME_WP = 2,
	CRITICAL_LAND = 3,
};

typedef struct  {
	imu_t imu;
	control_command_t controls;
	control_command_t controls_nav;

	stabiliser_stack_copter_t stabiliser_stack;

	simulation_model_t uav_model;
	servo_output_t servos[NUMBER_OF_SERVO_OUTPUTS];
	buffer_t xbee_in_buffer, wired_in_buffer;
	byte_stream_t xbee_out_stream;
	byte_stream_t xbee_in_stream;
	byte_stream_t wired_out_stream, wired_in_stream;
	
	buffer_t gps_buffer;
	byte_stream_t gps_stream_in;
	byte_stream_t gps_stream_out;
	gps_t gps;
	
	simulation_model_t sim_model;
	
	position_estimation_t position_estimation;
	
	// aliases
	byte_stream_t *telemetry_down_stream, *telemetry_up_stream;
	byte_stream_t *debug_out_stream, *debug_in_stream;
	
	waypoint_struct_t waypoint_list[MAX_WAYPOINTS];
	waypoint_struct_t current_waypoint;
	uint16_t number_of_waypoints;
	int8_t current_wp_count;
	
	local_coordinates_t waypoint_coordinates, waypoint_hold_coordinates, waypoint_critical_coordinates;
	float dist2wp_sqr;
	
	bool waypoint_set;
	bool waypoint_sending;
	bool waypoint_receiving;
	bool waypoint_handler_waypoint_hold_init;
	bool critical_landing;
	bool critical_next_state;
	
	bool collision_avoidance;
	bool automatic_take_off;
	
	uint8_t mav_mode;
	uint8_t mav_state;
	
	uint8_t mav_mode_previous;
	uint8_t mav_state_previous;
	
	uint32_t simulation_mode;
	
	barometer_t pressure;
	//float pressure_filtered;
	//float altitude_filtered;
	
	
	enum CRITICAL_BEHAVIOR_ENUM critical_behavior;
	
} central_data_t;


void central_data_init(void);

central_data_t* central_data_get_pointer_to_struct(void);

byte_stream_t* get_telemetry_upstream(void);
byte_stream_t* get_telemetry_downstream(void);

imu_t* get_imu_data();
control_command_t* get_control_inputs_data();

#define STDOUT &print_util_get_debug_stream()

#endif /* CENTRAL_DATA_H_ */