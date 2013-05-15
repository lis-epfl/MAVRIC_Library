/*
 * boardsupport.h
 *
 * Created: 20/03/2013 12:14:04
 *  Author: sfx
 */ 


#ifndef BOARDSUPPORT_H_
#define BOARDSUPPORT_H_


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
#include "estimator.h"

static const servo_output servo_failsafe[NUMBER_OF_SERVO_OUTPUTS]={{.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}, {.value=-600}};

typedef struct  {
	Imu_Data_t imu1;
	Control_Command_t controls;
	servo_output servos[NUMBER_OF_SERVO_OUTPUTS];
	byte_stream_t xbee_out_stream;
	byte_stream_t xbee_in_stream;
	byte_stream_t debug_stream;	
	
	Buffer_t gps_buffer;
	byte_stream_t gps_stream_in;
	byte_stream_t gps_stream_out;
	gps_Data_type GPS_data;
	
	Estimator_Data_t estimation;
	
	// aliases
	byte_stream_t *telemetry_down_stream, *telemetry_up_stream;
	
	
} board_hardware_t;


board_hardware_t* initialise_board();

board_hardware_t* get_board_hardware();

byte_stream_t* get_telemetry_upstream();
byte_stream_t* get_telemetry_downstream();
byte_stream_t* get_debug_stream();

Imu_Data_t* get_imu();
Control_Command_t* get_control_inputs();


#define STDOUT &get_debug_stream()
//#define STDOUT &xbee_out_stream


#endif /* BOARDSUPPORT_H_ */