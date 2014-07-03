/*
 * central_data.h
 *
 * Created: 12/11/2013 02:51:07
 *  Author: sfx
 */ 


#ifndef CENTRAL_DATA_H_
#define CENTRAL_DATA_H_

#include "time_keeper.h"
#include "i2c_driver_int.h"
#include "qfilter.h"
#include "imu.h"
#include "stabilisation.h"
#include "spektrum.h"
#include "streams.h"
#include "uart_int.h"
#include "print_util.h"

#include "bmp085.h"
#include "mavlink_stream.h"
#include "coord_conventions.h"
#include "onboard_parameters.h"
#include "servo_pwm.h"

#include "adc_int.h"
#include "dac_dma.h"


typedef struct  {

	int dummy[10];
	Buffer_t  wired_in_buffer;
	byte_stream_t wired_out_stream, wired_in_stream;
	
		
	// aliases
	byte_stream_t *telemetry_down_stream, *telemetry_up_stream;
	byte_stream_t *debug_out_stream, *debug_in_stream;

} central_data_t;

central_data_t* central_data_get_pointer_to_struct();

byte_stream_t* get_telemetry_upstream();
byte_stream_t* get_telemetry_downstream();


#define STDOUT &get_debug_stream()
//#define STDOUT &xbee_out_stream

#endif /* CENTRAL_DATA_H_ */