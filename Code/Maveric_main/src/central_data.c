/*
 * central_data.c
 *
 * Created: 16/09/2013 12:18:14
 *  Author: sfx
 */ 


#include "central_data.h"
#include "remote_controller.h"

static volatile central_data_t centralData;

central_data_t* initialise_central_data(){
	
}

central_data_t* get_central_data(void)
{
	return &centralData;
}

byte_stream_t* get_telemetry_upstream() {
	return centralData.telemetry_up_stream;
}
byte_stream_t* get_telemetry_downstream() {
	return centralData.telemetry_down_stream;
}
byte_stream_t* get_debug_stream() {
	return centralData.debug_out_stream;
}

Imu_Data_t* get_imu_data() {
	return &centralData.imu1;
}
Control_Command_t* get_control_inputs_data() {
	return &centralData.controls;
}