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
//#include "stdio_serial.h"
#include "print_util.h"
#include "generator.h"

#include "time_keeper.h"
#include "i2c_driver_int.h"
#include "qfilter.h"
#include "stabilisation.h"
#include "streams.h"

#include "bmp085.h"

#include "scheduler.h"
#include "boardsupport.h"

board_hardware_t *board;

pressure_data *pressure;


NEW_TASK_SET(main_tasks, 10)

void run_stabilisation() {
	board->controls.rpy[ROLL]=-getChannel(S_ROLL)/350.0;
	board->controls.rpy[PITCH]=-getChannel(S_PITCH)/350.0;
	board->controls.rpy[YAW]=getChannel(S_YAW)/350.0;
	board->controls.thrust=getChannel(S_THROTTLE)/350.0;

	if (getChannel(4)>0) {
		quad_stabilise(&(board->imu1), &(board->controls), RATE_COMMAND_MODE);
	} else {
		quad_stabilise(&(board->imu1), &(board->controls), ATTITUDE_COMMAND_MODE);
	}

}

void initialisation() {
	int i;
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
	
	board=initialise_board();
	
	if (init_i2c(0)!=STATUS_OK) {
		//putstring(STDOUT, "Error initialising I2C\n");
		while (1==1);
	} else {
		//putstring(STDOUT, "initialised I2C.\n");
	};

	
	Enable_global_interrupt();
		
	dbg_print("Debug stream initialised\n");

	LED_Off(LED1);
	
/*
	set_servo(0, -500, -500);
	set_servo(1, -500, -500);
	set_servo(2, -500, -500);
	set_servo(3, -500, -500);
*/
	
	set_servo(0, -600, -600);
	set_servo(1, -600, -600);
	set_servo(2, -600, -600);
	set_servo(3, -600, -600);
	
	//delay_ms(1000);
	init_stabilisation();

	init_onboard_parameters();
	
	board->imu1.attitude.calibration_level=LEVELING;	
	for (i=200; i>0; i--) {
		imu_update(&board->imu1);
		if (i%50 ==0) {
			// Send heartbeat message
			mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_ARMED, 0, MAV_STATE_CALIBRATING);
		}
		
		delay_ms(5);
	}
	board->imu1.attitude.calibration_level=OFF;
	
}

void main (void)
{
	int i=0;
	int counter=0;
	uint32_t last_looptime, this_looptime;

	initialisation();
	
	

	// main loop
	counter=0;
	while (1==1) {
		this_looptime=get_millis();
		

		
		imu_update(&(board->imu1));
		run_stabilisation();
		
				
		if(counter%200==0) {
			// Send a heartbeat over UART0 including the system type
			//mavlink_msg_heartbeat_send(mavlink_channel_t chan, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
			mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_ARMED, 0, MAV_STATE_ACTIVE);
		}
		
		if(counter%30==0) {
			// ATTITUDE QUATERNION
			mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0, 0, board->imu1.attitude.qe.s, board->imu1.attitude.qe.v[0], board->imu1.attitude.qe.v[1], board->imu1.attitude.qe.v[2], board->imu1.attitude.om[0], board->imu1.attitude.om[1], board->imu1.attitude.om[2]);
		
			// ATTITUDE
			Aero_Attitude_t aero_attitude;
			aero_attitude=Quat_to_Aero(board->imu1.attitude.qe);
			//mavlink_msg_attitude_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
			//mavlink_msg_attitude_send(MAVLINK_COMM_0, 0, aero_attitude.rpy[0], aero_attitude.rpy[1], aero_attitude.rpy[2], imu1.attitude.om[0], imu1.attitude.om[1], imu1.attitude.om[2]);
			
			Schill_Attitude_t schill_attitude;
			schill_attitude=Quat_to_Schill(board->imu1.attitude.qe);
			//mavlink_msg_attitude_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
			mavlink_msg_attitude_send(MAVLINK_COMM_0, 0, schill_attitude.rpy[0], schill_attitude.rpy[1], schill_attitude.rpy[2], board->imu1.attitude.om[0], board->imu1.attitude.om[1], board->imu1.attitude.om[2]);
			
			// Controls output
			//mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float thrust)
			mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(MAVLINK_COMM_0, 0, board->controls.rpy[ROLL], board->controls.rpy[PITCH], board->controls.rpy[YAW], board->controls.thrust);
			
			// GPS COORDINATES (TODO : Add GPS to the platform)
			//mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
			mavlink_msg_global_position_int_send(MAVLINK_COMM_0, 0, 46.5193*10000000, 6.56507*10000000, 400, 1, 0, 0, 0, board->imu1.attitude.om[2]);

			mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Compass_X", board->imu1.raw_channels[COMPASS_OFFSET+IMU_X]);
			mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Compass_Y", board->imu1.raw_channels[COMPASS_OFFSET+IMU_Y]);
			mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Compass_Z", board->imu1.raw_channels[COMPASS_OFFSET+IMU_Z]);

			// NAMED VALUES
			//mavlink_msg_named_value_float_send(mavlink_channel_t chan, uint32_t time_boot_ms, const char *name, float value)
			mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "LoopTime", this_looptime-last_looptime);
			//mavlink_msg_named_value_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, const char *name, int32_t value
			mavlink_msg_named_value_int_send(MAVLINK_COMM_0, 0, "User_val_2", 201);
			
			//pressure=get_pressure_data_slow();
			//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Pressure", pressure->pressure);
			//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Temperature", pressure->temperature);
			//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Altitude", pressure->altitude);
			
		}
		
		if(counter%30==0) {	

		}
		
		LED_On(LED1);
		delay_ms(1);
		counter=(counter+1)%1000;
		last_looptime=this_looptime;	
	}		
}


