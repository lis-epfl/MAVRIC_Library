/*
 * boardsupport.h
 *
 * Created: 20/03/2013 12:14:04
 *  Author: sfx
 * 
 * A place for all central data structures, system-specific initialisation and access methods. 
 * To do: Maybe should be renamed - system_support.h ?
 */ 


#ifndef BOARDSUPPORT_H_
#define BOARDSUPPORT_H_

#include "stdbool.h"

#include "time_keeper.h"
#include "i2c_driver_int.h"
#include "qfilter.h"
#include "imu.h"
#include "stabilisation.h"
#include "spektrum.h"
#include "control.h"
#include "streams.h"
#include "uart_int.h"
#include "print_util.h"

#include "bmp085.h"
#include "mavlink_stream.h"
#include "coord_conventions.h"
#include "onboard_parameters.h"
#include "servo_pwm.h"

#include "gps_ublox.h"
#include "waypoint_navigation.h"
#include "estimator.h"
#include "simulation.h"

static const servo_output servo_failsafe[NUMBER_OF_SERVO_OUTPUTS]={{.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}};

typedef struct  {
	Imu_Data_t imu1;
	Control_Command_t controls;
	simulation_model_t uav_model;
	servo_output servos[NUMBER_OF_SERVO_OUTPUTS];
	Buffer_t xbee_in_buffer, wired_in_buffer;
	byte_stream_t xbee_out_stream;
	byte_stream_t xbee_in_stream;
	byte_stream_t wired_out_stream, wired_in_stream;	
	
	Buffer_t gps_buffer;
	byte_stream_t gps_stream_in;
	byte_stream_t gps_stream_out;
	gps_Data_type GPS_data;
	
	Estimator_Data_t estimation;
	simulation_model_t sim_model;
	
	local_coordinates_t local_position;
	bool init_gps_position;
	
	// aliases
	byte_stream_t *telemetry_down_stream, *telemetry_up_stream;
	byte_stream_t *debug_out_stream, *debug_in_stream;	
	
	waypoint_struct waypoint_list[MAX_WAYPOINTS];
	uint16_t number_of_waypoints;
	int8_t current_wp;
	
	uint8_t mav_mode;
	uint8_t mav_state;
	uint8_t simulation_mode;
	
} board_hardware_t;


board_hardware_t* initialise_board(void);

board_hardware_t* get_board_hardware(void);

byte_stream_t* get_telemetry_upstream(void);
byte_stream_t* get_telemetry_downstream(void);
byte_stream_t* get_debug_stream(void);

Imu_Data_t* get_imu(void);
Control_Command_t* get_control_inputs(void);


#define STDOUT &get_debug_stream()
//#define STDOUT &xbee_out_stream


#endif /* BOARDSUPPORT_H_ */