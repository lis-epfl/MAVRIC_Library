/*
 * boardsupport.c
 *
 * Created: 20/03/2013 12:14:18
 *  Author: Felix Schill
 */ 

#include "boardsupport.h"
#include "waypoint_navigation.h"
#include "conf_sim_model.h"
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include "udp_stream.h"

udp_connection_t udp_out, udp_in;

static volatile board_hardware_t board_hardware;

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



board_hardware_t* initialise_board() {
//		init_UART_int(0);

		make_buffered_stream(&board_hardware.xbee_in_buffer, &board_hardware.xbee_in_stream);

		//register_read_stream_stdin( &board_hardware.xbee_in_stream);
		register_read_stream_udp( &board_hardware.xbee_in_stream, &udp_in, 14551);
		udp_out.sock=udp_in.sock;

//		register_write_stream(get_UART_handle(0), &board_hardware.xbee_out_stream);
		//register_write_stream_stdout( &board_hardware.xbee_out_stream);		
		register_write_stream_udp(&board_hardware.xbee_out_stream, &udp_out, "127.0.0.1",14550);
		
//		init_UART_int(3);
		make_buffered_stream(&(board_hardware.gps_buffer), &board_hardware.gps_stream_in);
//		register_read_stream(get_UART_handle(3), &board_hardware.gps_stream_in);
//		register_write_stream(get_UART_handle(3), &board_hardware.gps_stream_out);
		
//		init_UART_int(4);

//		register_write_stream(get_UART_handle(4), &board_hardware.wired_out_stream);
		register_write_stream_stdout( &board_hardware.wired_out_stream);
		
		// connect abstracted aliases to hardware ports


		board_hardware.telemetry_down_stream=&board_hardware.xbee_out_stream;
		board_hardware.telemetry_up_stream=&board_hardware.xbee_in_stream;
		board_hardware.debug_out_stream=&board_hardware.wired_out_stream;
		//board_hardware.debug_out_stream=NULL;
		//board_hardware.debug_in_stream=&board_hardware.wired_in_stream;
/*
		board_hardware.telemetry_down_stream=&board_hardware.wired_out_stream;
		board_hardware.telemetry_up_stream  =&board_hardware.wired_in_stream;		
		board_hardware.debug_out_stream     =&board_hardware.xbee_out_stream;
		board_hardware.debug_in_stream      =&board_hardware.xbee_in_stream;
*/
		// init mavlink
		init_mavlink(board_hardware.telemetry_down_stream, board_hardware.telemetry_up_stream, 42);
		
//		register_read_stream(get_UART_handle(4), &board_hardware.wired_in_stream);
//		register_read_stream(get_UART_handle(0), &board_hardware.xbee_in_stream);
		
		

		// init debug output
		dbg_print_init(board_hardware.debug_out_stream);
		
		init_imu(&board_hardware.imu1);

		rc_init();
		init_Servos();
		init_simulation(&board_hardware.sim_model);
		
		board_hardware.controls.rpy[ROLL]=0;
		board_hardware.controls.rpy[PITCH]=0;
		board_hardware.controls.rpy[YAW]=0;
		board_hardware.controls.thrust=-1.0;
		
		board_hardware.number_of_waypoints = 0;
		board_hardware.waypoint_hold_init=false;
		board_hardware.simulation_mode=0;
		
		// default GPS home position
		board_hardware.imu1.attitude.localPosition.origin.longitude=   HOME_LONGITUDE;
		board_hardware.imu1.attitude.localPosition.origin.latitude =   HOME_LATITUDE;
		board_hardware.imu1.attitude.localPosition.origin.altitude =   HOME_ALTITUDE;
		board_hardware.imu1.attitude.localPosition.pos[0]=0;	board_hardware.imu1.attitude.localPosition.pos[1]=0; board_hardware.imu1.attitude.localPosition.pos[2]=0;
		
		init_waypoint_list(board_hardware.waypoint_list,&board_hardware.number_of_waypoints);

		return &board_hardware;
}

board_hardware_t* get_board_hardware() {
	return &board_hardware;
}


byte_stream_t* get_telemetry_upstream() {
	return board_hardware.telemetry_up_stream;
}
byte_stream_t* get_telemetry_downstream() {
	return board_hardware.telemetry_down_stream;
}
byte_stream_t* get_debug_stream() {
	return board_hardware.debug_out_stream;
}

Imu_Data_t* get_imu() {
	return &board_hardware.imu1;
}
Control_Command_t* get_control_inputs() {
	return &board_hardware.controls;
}
