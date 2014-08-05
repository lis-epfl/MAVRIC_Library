/*
 * boardsupport.c
 *
 * Created: 20/03/2013 12:14:18
 *  Author: Felix Schill
 */ 

#include "boardsupport.h"
#include "mavlink_waypoint_handler.h"
#include "conf_sim_model.h"
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include "udp_stream.h"

udp_connection_t udp_out, udp_in;

static volatile central_data_t central_data;

static inline void stdout_send_byte(void* options, char data) {
	//printf("%c", data);
	char temp = data;
	fwrite(&temp, 1, 1, stdout);
	fflush(stdout);
}

static inline void stdout_flush(void* options) {
	fflush(stdout);
}
static inline bool stdout_buffer_empty(void* options){
	return true;
}

// dummy uart outputs via printf to console
void register_write_stream_stdout(byte_stream_t *stream) {
	stream->get=NULL;
	//stream->get=&uart_int_get_byte;
	stream->put=&stdout_send_byte;
	stream->flush=&stdout_flush;
	stream->buffer_empty=&stdout_buffer_empty;
	stream->data=NULL;

}


bool inputAvailable()  
{
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return (FD_ISSET(0, &fds));
}

bool fill_input_buffer(buffer_t *buffer) {
	char buf;
	int32_t n;
	if (inputAvailable()){
		n=read(STDIN_FILENO, &buf, 1);
		if (n>0) buffer_put(buffer, buf);
	}	
	return buffer_bytes_available(buffer);
	
}

void register_read_stream_stdin( byte_stream_t *stream) {
	stream->bytes_available=&fill_input_buffer;
	//stream->get=&read_stdin;
}



void boardsupport_init(central_data_t *central_data) {
//		uart_int_init(0);

		buffer_make_buffered_stream(&central_data->xbee_in_buffer, &central_data->xbee_in_stream);

		//register_read_stream_stdin( &central_data->xbee_in_stream);
		
		//register_read_stream_udp( &central_data->xbee_in_stream, &udp_in, 14551);
		//udp_out.sock=udp_in.sock;

//		uart_int_register_write_stream(uart_int_get_uart_handle(0), &central_data->xbee_out_stream);
		//register_write_stream_stdout( &central_data->xbee_out_stream);		
		//register_write_stream_udp(&central_data->xbee_out_stream, &udp_out, "127.0.0.1",14550);
		
//		uart_int_init(3);
		buffer_make_buffered_stream(&(central_data->gps_buffer), &central_data->gps_stream_in);
//		uart_int_register_read_stream(uart_int_get_uart_handle(3), &central_data->gps_stream_in);
//		uart_int_register_write_stream(uart_int_get_uart_handle(3), &central_data->gps_stream_out);
		
//		uart_int_init(4);

//		uart_int_register_write_stream(uart_int_get_uart_handle(4), &central_data->wired_out_stream);
		register_write_stream_stdout( &central_data->wired_out_stream);
		
		// connect abstracted aliases to hardware ports


		central_data->telemetry_down_stream=&central_data->xbee_out_stream;
		central_data->telemetry_up_stream=&central_data->xbee_in_stream;
		central_data->debug_out_stream=&central_data->wired_out_stream;
		//central_data->debug_out_stream=NULL;
		//central_data->debug_in_stream=&central_data->wired_in_stream;
/*
		central_data->telemetry_down_stream=&central_data->wired_out_stream;
		central_data->telemetry_up_stream  =&central_data->wired_in_stream;		
		central_data->debug_out_stream     =&central_data->xbee_out_stream;
		central_data->debug_in_stream      =&central_data->xbee_in_stream;
*/
		// init mavlink
		mavlink_stream_init(central_data->telemetry_down_stream, central_data->telemetry_up_stream, 42);
		
//		uart_int_register_read_stream(uart_int_get_uart_handle(4), &central_data->wired_in_stream);
//		uart_int_register_read_stream(uart_int_get_uart_handle(0), &central_data->xbee_in_stream);
		
		

		// init debug output
		print_util_dbg_print_init(central_data->debug_out_stream);
		
		imu_init(&central_data->imu);

		spektrum_satellite_init();
		servo_pwm_init_old();
		
		
		central_data->controls.rpy[ROLL]=0;
		central_data->controls.rpy[PITCH]=0;
		central_data->controls.rpy[YAW]=0;
		central_data->controls.thrust=-1.0;
		
		central_data->number_of_waypoints = 0;
		central_data->waypoint_handler_waypoint_hold_init=false;
		central_data->simulation_mode=0;
		
		// default GPS home position
		central_data->position_estimator.local_position.origin.longitude=   HOME_LONGITUDE;
		central_data->position_estimator.local_position.origin.latitude =   HOME_LATITUDE;
		central_data->position_estimator.local_position.origin.altitude =   HOME_ALTITUDE;
		central_data->position_estimator.local_position.pos[0]=0;	central_data->position_estimator.local_position.pos[1]=0; central_data->position_estimator.local_position.pos[2]=0;
		
		simulation_init(&central_data->sim_model, &central_data->imu.attitude);
		central_data->sim_model.local_position=central_data->position_estimator.local_position;


		return &central_data;
}


