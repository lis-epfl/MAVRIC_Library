/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * AVR Software Framework (ASF).
 */
#include <asf.h>
#include "sysclk.h"
#include "sleepmgr.h"
#include "led.h"
#include "delay.h"
#include "spi_buffered.h"
//#include "stdio_serial.h"
#include "print_util.h"
#include "generator.h"
#include "servo_pwm.h"

#include "time_keeper.h"
#include "i2c_driver_int.h"
#include "qfilter.h"
#include "imu.h"
#include "stabilisation.h"
#include "spektrum.h"
#include "control.h"
#include "streams.h"
#include "uart_int.h"

#include "ishtar_stream.h"
#include "mavlink_stream.h"
#include "coord_conventions.h"

Imu_Data_t imu1;

Control_Command_t controls;

byte_stream_t xbee_stream;
byte_stream_t debug_stream;

#define STDOUT &debug_stream
//#define STDOUT &xbee_stream


void main (void)
{
	int i=0;
	int counter=0;
	uint32_t last_looptime, this_looptime;
	irq_initialize_vectors();
	cpu_irq_enable();
	Disable_global_interrupt();
	
	// Initialize the sleep manager
	sleepmgr_init();

	sysclk_init();
	board_init();
	delay_init(sysclk_get_cpu_hz());
	init_time_keeper();

	
	INTC_init_interrupts();
	if (init_i2c(0)!=STATUS_OK) {
//		putstring(STDOUT, "Error initialising I2C\n");
		while (1==1);
	} else {
//		putstring(STDOUT, "initialised I2C.\n");
	};

	spektrum_init();
	
	init_UART_int(0);
	register_write_stream(get_UART_handle(0), &xbee_stream);
	
	init_UART_int(4);
	register_write_stream(get_UART_handle(4), &debug_stream);

	// init mavlink
	init_mavlink(&xbee_stream);
	register_read_stream(get_UART_handle(0), mavlink_in_stream);	
		
	Enable_global_interrupt();
	//print_init();
	//delay_ms(100);
	
	//putstring(&xbee_stream, "Initialised xbeestream\n");
	
	print_init(XBEE_UART_ID);
	dbg_print("Debug stream initialised\n");

	LED_Off(LED1);
	
	init_Servos();
	set_servo(0, -600, -600);
	set_servo(1, -600, -600);
	set_servo(2, -600, -600);
	set_servo(3, -600, -600);
	
	//delay_ms(1000);
	init_imu(&imu1);
	imu1.attitude.calibration_level=LEVELING;
	init_stabilisation();

	controls.rpy[ROLL]=0;
	controls.rpy[PITCH]=0;
	controls.rpy[YAW]=0;
	controls.thrust=-1.0;
	
	for (i=200; i>0; i--) {
		imu_update(&imu1);
		if (i%100 ==0) {
			// Send heartbeat message
			// mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_ARMED, 0, MAV_STATE_CALIBRATING);
		}
				
		delay_ms(5);
	}
	imu1.attitude.calibration_level=OFF;

	// main loop
	counter=0;
	while (1==1) {
		this_looptime=get_millis();
		
		controls.rpy[ROLL]=-getChannel(S_ROLL)/350.0;
		controls.rpy[PITCH]=-getChannel(S_PITCH)/350.0;
		controls.rpy[YAW]=getChannel(S_YAW)/350.0;
		controls.thrust=getChannel(S_THROTTLE)/350.0;
		
		imu_update(&imu1);
			
		if (getChannel(4)>0) {
			quad_stabilise(&imu1, &controls, RATE_COMMAND_MODE);
		} else {
			quad_stabilise(&imu1, &controls, ATTITUDE_COMMAND_MODE);
		}		
		
		if (counter%20==0) {
			/*
			if (checkReceivers()>=0) {
				putstring(STDOUT, "R");
			}
			if (getChannel(4)>0) {
				putstring(STDOUT, " RCM ");
			}else {
				putstring(STDOUT, " ACM ");
			}				
			
			for (i=0; i<6; i++) {
				putnum(STDOUT, imu1.raw_channels[i], 10);
				//putnum(STDOUT, (int)(1*imu1.attitude.om[i]), 10);
				putstring(STDOUT, "\t");
			}	
		
			for (i=0; i<3; i++) {
				putnum(STDOUT, (int)(1000.0*imu1.attitude.a[i]), 10);
				putstring(STDOUT, "\t");
			}
		
			putstring(STDOUT, "S");
			for (i=0; i<3; i++) {
				putnum(STDOUT, stabiliser.output.rpy[i]*100, 10);
				putstring(STDOUT, "\t");
			}
			putnum(STDOUT, stabiliser.output.thrust+*100, 10);
	
	
			for (i=0; i<4; i++) {
				putnum(STDOUT, getChannel(i), 10);
				putstring(STDOUT, "\t");
			}
	
		
			for (i=0; i<3; i++) {
				putnum(STDOUT, (int)(100.0*imu1.attitude.up_vec.v[i]), 10);
				putstring(STDOUT, "\t");
			}
				
			for (i=0; i<3; i++) {
				putnum(STDOUT, (int)(100.0*get_rate_stabiliser()->output.rpy[i]), 10);
				putstring(STDOUT, ",");
				putnum(STDOUT, (int)(100.0*get_rate_stabiliser()->rpy_controller[i].integrator.accumulator), 10);

				putstring(STDOUT, "\t");
			}	
		
			putnum(STDOUT, imu1.dt*1000, 10);
			
			putstring(STDOUT, "\n");
			putstring(STDOUT, "Controls :");
			putstring(STDOUT, "\t");
			putnum(STDOUT, 100*controls.rpy[ROLL], 10);
			putstring(STDOUT, "\t");
			putnum(STDOUT, 100*controls.rpy[PITCH], 10);
			putstring(STDOUT, "\t");
			putnum(STDOUT, 100*controls.rpy[YAW], 10);
			putstring(STDOUT, "\t");
			putnum(STDOUT, 100*controls.thrust, 10);
			putstring(STDOUT, "\n");
			*/
			
			
		}
		
		/*if(counter%300==0) {
			putstring(STDOUT, "Toggle ! \n");
			LED_Toggle(LED1);
		}*/
				
		if(counter%300==0) {
			// Send a heartbeat over UART0 including the system type
			//mavlink_msg_heartbeat_send(mavlink_channel_t chan, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
			mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_ARMED, 0, MAV_STATE_ACTIVE);
		}
		
		if(counter%30==0) {
			// ATTITUDE QUATERNION
			//mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0, 0, imu1.attitude.qe.s, imu1.attitude.qe.v[0], imu1.attitude.qe.v[1], imu1.attitude.qe.v[2], imu1.attitude.om[0], imu1.attitude.om[1], imu1.attitude.om[2]);
		
			// ATTITUDE
			/*Aero_Attitude_t aero_attitude;
			aero_attitude=Quat_to_Aero(imu1.attitude.qe);
			//mavlink_msg_attitude_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
			mavlink_msg_attitude_send(MAVLINK_COMM_0, 0, aero_attitude.rpy[0], aero_attitude.rpy[1], aero_attitude.rpy[2], imu1.attitude.om[0], imu1.attitude.om[1], imu1.attitude.om[2]);
			*/
			Schill_Attitude_t schill_attitude;
			schill_attitude=Quat_to_Schill(imu1.attitude.qe);
			//mavlink_msg_attitude_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
			mavlink_msg_attitude_send(MAVLINK_COMM_0, 0, schill_attitude.rpy[0], schill_attitude.rpy[1], schill_attitude.rpy[2], imu1.attitude.om[0], imu1.attitude.om[1], imu1.attitude.om[2]);
		
			// GPS COORDINATES (TODO : Add GPS to the platform)
			//mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
			mavlink_msg_global_position_int_send(MAVLINK_COMM_0, 0, 46.5193*10000000, 6.56507*10000000, 400, 1, 0, 0, 0, imu1.attitude.om[2]);

			// NAMED VALUES
			//mavlink_msg_named_value_float_send(mavlink_channel_t chan, uint32_t time_boot_ms, const char *name, float value)
			mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "LoopTime", this_looptime-last_looptime);
			//mavlink_msg_named_value_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, const char *name, int32_t value
			//mavlink_msg_named_value_int_send(MAVLINK_COMM_0, 0, "User_val_2", 201);
			
			// Test receive stream
			putnum(&debug_stream,(long)buffer_bytes_available(mavlink_in_stream->data), 10);
		}

		
		
		LED_On(LED1);
		delay_ms(1);
		counter=(counter+1)%1000;
		last_looptime=this_looptime;	

	}		
	
	
}


