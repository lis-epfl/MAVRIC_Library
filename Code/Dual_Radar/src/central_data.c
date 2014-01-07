/*
 * central_data.c
 *
 * Created: 12/11/2013 02:52:18
 *  Author: sfx
 */ 

#include "central_data.h"

static volatile central_data_t central_data;


central_data_t* get_central_data() {
	return &central_data;
}

byte_stream_t* get_telemetry_upstream() {
	return central_data.telemetry_up_stream;
}
byte_stream_t* get_telemetry_downstream() {
	return central_data.telemetry_down_stream;
}
byte_stream_t* get_debug_stream() {
	return central_data.debug_out_stream;
}
