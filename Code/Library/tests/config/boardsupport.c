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

static volatile central_data_t centralData;

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

bool fill_input_buffer(Buffer_t *buffer) {
	char buf;
	int n;
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



void initialise_board(central_data_t *centralData) {
//		init_UART_int(0);

		make_buffered_stream(&centralData->xbee_in_buffer, &centralData->xbee_in_stream);

		//register_read_stream_stdin( &centralData->xbee_in_stream);
		
		//register_read_stream_udp( &centralData->xbee_in_stream, &udp_in, 14551);
		//udp_out.sock=udp_in.sock;

//		register_write_stream(get_UART_handle(0), &centralData->xbee_out_stream);
		//register_write_stream_stdout( &centralData->xbee_out_stream);		
		//register_write_stream_udp(&centralData->xbee_out_stream, &udp_out, "127.0.0.1",14550);
		
//		init_UART_int(3);
		make_buffered_stream(&(centralData->gps_buffer), &centralData->gps_stream_in);
//		register_read_stream(get_UART_handle(3), &centralData->gps_stream_in);
//		register_write_stream(get_UART_handle(3), &centralData->gps_stream_out);
		
//		init_UART_int(4);

//		register_write_stream(get_UART_handle(4), &centralData->wired_out_stream);
		register_write_stream_stdout( &centralData->wired_out_stream);
		
		// connect abstracted aliases to hardware ports


		centralData->telemetry_down_stream=&centralData->xbee_out_stream;
		centralData->telemetry_up_stream=&centralData->xbee_in_stream;
		centralData->debug_out_stream=&centralData->wired_out_stream;
		//centralData->debug_out_stream=NULL;
		//centralData->debug_in_stream=&centralData->wired_in_stream;
/*
		centralData->telemetry_down_stream=&centralData->wired_out_stream;
		centralData->telemetry_up_stream  =&centralData->wired_in_stream;		
		centralData->debug_out_stream     =&centralData->xbee_out_stream;
		centralData->debug_in_stream      =&centralData->xbee_in_stream;
*/
		// init mavlink
		init_mavlink(centralData->telemetry_down_stream, centralData->telemetry_up_stream, 42);
		
//		register_read_stream(get_UART_handle(4), &centralData->wired_in_stream);
//		register_read_stream(get_UART_handle(0), &centralData->xbee_in_stream);
		
		

		// init debug output
		dbg_print_init(centralData->debug_out_stream);
		
		init_imu(&centralData->imu1);

		rc_init();
		init_Servos();
		
		
		centralData->controls.rpy[ROLL]=0;
		centralData->controls.rpy[PITCH]=0;
		centralData->controls.rpy[YAW]=0;
		centralData->controls.thrust=-1.0;
		
		centralData->number_of_waypoints = 0;
		centralData->waypoint_hold_init=false;
		centralData->simulation_mode=0;
		
		// default GPS home position
		centralData->position_estimator.localPosition.origin.longitude=   HOME_LONGITUDE;
		centralData->position_estimator.localPosition.origin.latitude =   HOME_LATITUDE;
		centralData->position_estimator.localPosition.origin.altitude =   HOME_ALTITUDE;
		centralData->position_estimator.localPosition.pos[0]=0;	centralData->position_estimator.localPosition.pos[1]=0; centralData->position_estimator.localPosition.pos[2]=0;
		
		init_simulation(&centralData->sim_model, &centralData->imu1.attitude);
		centralData->sim_model.localPosition=centralData->position_estimator.localPosition;


		return &centralData;
}


