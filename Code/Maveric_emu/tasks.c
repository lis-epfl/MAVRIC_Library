/*
 * tasks.c
 *
 * Created: 16/09/2013 13:28:11
 *  Author: sfx
 */ 


#include "tasks.h"
#include "boardsupport.h"
#include "print_util.h"
#include "stabilisation.h"
#include "gps_ublox.h"
#include "estimator.h"
#include "navigation.h"

NEW_TASK_SET(main_tasks, 10)

#define PRESSURE_LPF 0.1

board_hardware_t *board;


task_set_t* tasks_get_main_taskset() {
	return &main_tasks;
}

task_return_t tasks_run_imu_update() {
	imu_update(&(board->imu1));	
}	

void tasks_rc_user_channels(uint8_t *chanSwitch, int8_t *rc_check, int8_t *motorbool)
{
	
	remote_controller_get_channel_mode(chanSwitch);
	
	//print_util_dbg_print("chanSwitch ");
	//print_util_dbg_print_num(*chanSwitch,10);
	//print_util_dbg_print_num(getChannel(4),10);
	//print_util_dbg_print_num(getChannel(5),10);
	//print_util_dbg_print("\n");
	
	if((remote_controller_get_thrust_from_remote()<-0.95) && (remote_controller_get_yaw_from_remote() > 0.9))
	{
		//print_util_dbg_print("motor on\n");
		*motorbool = 1;
	}else if((remote_controller_get_thrust_from_remote()<-0.95) && (remote_controller_get_yaw_from_remote() <-0.9))
	{
		//print_util_dbg_print("motor off\n");
		*motorbool = -1;
	}else{
		//print_util_dbg_print("motor nothing\n");
		*motorbool = 0;
	}
	
	switch (remote_dsm2_rc_check_receivers())
	{
		case 1:
		*rc_check = 1;
		break;
		case -1:
		*rc_check = -1;
		break;
		case -2:
		*rc_check = -2;
		break;
	}
	//print_util_dbg_print("rc_check: ");
	//print_util_dbg_print_num(rc_check,10);
	//print_util_dbg_print("; motorbool : ");
	//print_util_dbg_print_num(*motorbool,10);
	//print_util_dbg_print("\n");
}

task_return_t tasks_set_mav_mode_n_state()
{
	uint8_t channelSwitches = 0;
	int8_t RC_check = 0;
	int8_t motor_switch = 0;
	
	tasks_rc_user_channels(&channelSwitches,&RC_check, &motor_switch);
	
	switch(board->mav_state)
	{
		case MAV_STATE_CALIBRATING:
			break;
		case MAV_STATE_STANDBY:
			if (motor_switch == 1)
			{
				switch(channelSwitches)
				{
					case 0:
						print_util_dbg_print("Switching on the motors!\n");
						board->controls.run_mode = MOTORS_ON;
						board->mav_state = MAV_STATE_ACTIVE;
						board->mav_mode = MAV_MODE_MANUAL_ARMED;
						break;
					case 1:
						print_util_dbg_print("Switches not ready, both should be pushed!\n");
						//board->controls.run_mode = MOTORS_ON;
						//board->mav_state = MAV_STATE_ACTIVE;
						//board->mav_mode = MAV_MODE_STABILIZE_ARMED;
						break;
					case 2:
						print_util_dbg_print("Switches not ready, both should be pushed!\n");
						break;
					case 3:
						print_util_dbg_print("Switches not ready, both should be pushed!\n");
						break;
				}
			}
			break;
		case MAV_STATE_ACTIVE:
			switch(channelSwitches)
			{
				case 0:
					board->mav_mode= MAV_MODE_STABILIZE_ARMED;
					break;
				case 1:
					board->mav_mode= MAV_MODE_STABILIZE_ARMED;
					break;
				case 2:
					board->mav_mode = MAV_MODE_GUIDED_ARMED;
					break;
				case 3:
					board->mav_mode = MAV_MODE_AUTO_ARMED;
					break;
			}
			//print_util_dbg_print("motor_switch: ");
			//print_util_dbg_print_num(motor_switch,10);
			if (motor_switch == -1)
			{
				print_util_dbg_print("Switching off motors!\n");
				board->controls.run_mode = MOTORS_OFF;
				board->mav_state = MAV_STATE_STANDBY;
				board->mav_mode = MAV_MODE_MANUAL_DISARMED;
			}
		
			switch (RC_check)
			{
				case 1:
					break;
				case -1:
					board->mav_state = MAV_STATE_CRITICAL;
					break;
				case -2:
					board->mav_state = MAV_STATE_EMERGENCY;
					break;
			}
			break;
		case MAV_STATE_CRITICAL:
			switch(channelSwitches)
			{
				case 0:
				board->mav_mode= MAV_MODE_STABILIZE_ARMED;
				break;
				case 1:
				board->mav_mode= MAV_MODE_STABILIZE_ARMED;
				break;
				case 2:
				board->mav_mode = MAV_MODE_GUIDED_ARMED;
				break;
				case 3:
				break;
			}
			if (motor_switch == -1)
			{
				print_util_dbg_print("Switching off motors!\n");
				board->controls.run_mode = MOTORS_OFF;
				board->mav_state = MAV_STATE_STANDBY;
				board->mav_mode = MAV_MODE_MANUAL_DISARMED;
			}
			switch (RC_check)
			{
				case 1:
				board->mav_state = MAV_STATE_ACTIVE;
				break;
				case -1:
				break;
				case -2:
				board->mav_state = MAV_STATE_EMERGENCY;
				break;
			}
			break;
		case MAV_STATE_EMERGENCY:
			board->mav_mode = MAV_MODE_MANUAL_DISARMED;
			if (board->imu1.attitude.localPosition.pos[Z] < 0.5)
			{
				board->mav_state = MAV_STATE_STANDBY;
			}
			break;
	}

	//print_util_dbg_print("MAV state :");
	//print_util_dbg_print_num(board->mav_state,10);
	//print_util_dbg_print(", MAV mode :");
	//print_util_dbg_print_num(board->mav_mode,10);
	//print_util_dbg_print("\n");
	
}

task_return_t tasks_run_stabilisation() {
	int i;
	
	if (board->simulation_mode==1) {
		simulation_update(&(board->sim_model), &(board->servos), &(board->imu1));
	} else {
		imu_update(&(board->imu1));
	}
	//board->local_position.pos[0] = board->imu1.attitude.pos[0];
	//board->local_position.pos[1] = board->imu1.attitude.pos[1];
	//board->local_position.pos[2] = board->imu1.attitude.pos[2];
	
	switch(board->mav_mode)
	{
		case MAV_MODE_PREFLIGHT:
		case MAV_MODE_MANUAL_ARMED:
			//board->controls = remote_controller_get_command_from_remote();
			//for (i=0; i<4; i++) {
			//	board->servos[i].value=SERVO_SCALE*board->controls.thrust;
			//}
			
			break;
		case MAV_MODE_STABILIZE_ARMED:
			board->waypoint_handler_waypoint_hold_init = false;
			board->mission_started = false;
			board->controls = remote_controller_get_command_from_remote();
			//print_util_dbg_print("Thrust:");
			//print_util_dbg_print_num(board->controls.thrust*10000,10);
			//print_util_dbg_print("\n");
			//board->controls.control_mode=ATTITUDE_COMMAND_MODE_REL_YAW;
			board->controls.tvel[X]=-10.0*board->controls.rpy[PITCH];
			board->controls.tvel[Y]= 10.0*board->controls.rpy[ROLL];
			board->controls.tvel[Z]=- 3.0*board->controls.thrust;
			board->controls.control_mode=VELOCITY_COMMAND_MODE;
			quad_stabilise(&(board->imu1), &(board->controls));
			break;
		case MAV_MODE_GUIDED_ARMED:
			board->mission_started = false;
			board->controls = board->controls_nav;
			board->controls.thrust = min(remote_controller_get_thrust_from_remote()*100000.0,board->controls_nav.thrust*100000.0)/100000.0;
			//board->controls.thrust = board->controls_nav.thrust;
			
			//print_util_dbg_print("Thrust (x10000):");
			//print_util_dbg_print_num(board->controls.thrust*10000.0,10);
			//print_util_dbg_print(", remote (x10000):");
			//print_util_dbg_print_num(remote_controller_get_thrust_from_remote()*10000.0,10);
			//print_util_dbg_print(" => min (x10000):");
			//print_util_dbg_print_num(min(remote_controller_get_thrust_from_remote()*100000.0,board->controls_nav.thrust*100000.0)/100000.0 *10000.0,10);
			//print_util_dbg_print("\n");
			
			quad_stabilise(&(board->imu1), &(board->controls));
			break;
		case MAV_MODE_AUTO_ARMED:
			board->mission_started = true;
			board->waypoint_handler_waypoint_hold_init = false;
			board->controls = board->controls_nav;
			board->controls.thrust = min(remote_controller_get_thrust_from_remote()*100000.0,board->controls_nav.thrust*100000.0)/100000.0;
			//board->controls.thrust = board->controls_nav.thrust;
			
			//print_util_dbg_print("Thrust main:");
			//print_util_dbg_print_num(board->controls.thrust*10000,10);
			//print_util_dbg_print("\n");
			
			quad_stabilise(&(board->imu1), &(board->controls));
			break;
		case MAV_MODE_MANUAL_DISARMED:
		case MAV_MODE_STABILIZE_DISARMED:
		case MAV_MODE_GUIDED_DISARMED:
		case MAV_MODE_AUTO_DISARMED:
			//servo_pwm_set(&(servo_failsafe));
			for (i=0; i<NUMBER_OF_SERVO_OUTPUTS; i++) {
				board->servos[i]=servo_failsafe[i];
			}
			break;
		
	}
	
	// !!! -- for safety, this should remain the only place where values are written to the servo outputs! --- !!!
	if (board->simulation_mode!=1) {
		servo_pwm_set(&(board->servos));
	}
		

}

//task_return_t tasks_run_stabilisation() {
	//board->controls.rpy[ROLL]=-getChannel(S_ROLL)/350.0;
	//board->controls.rpy[PITCH]=-getChannel(S_PITCH)/350.0;
	//board->controls.rpy[YAW]=-getChannel(S_YAW)/350.0;
	//board->controls.thrust=getChannel(S_THROTTLE)/350.0;
//
	//imu_update(&(board->imu1));
	//if (getChannel(4)>0) {
		//board->controls.control_mode=RATE_COMMAND_MODE;
	//} else {
		//board->controls.control_mode=ATTITUDE_COMMAND_MODE;
	//}
	//
	//// switch run_mode
	//if ((board->controls.thrust<-0.95) && (board->controls.rpy[YAW]< -0.9)) {
		//board->controls.run_mode=MOTORS_OFF;
		//LED_Off(LED1);
	//}
	//if ((board->controls.thrust<-0.95) && (board->controls.rpy[YAW] >0.9)) {
		//board->controls.run_mode=MOTORS_ON;
		//LED_On(LED1);
	//}
		//
	//quad_stabilise(&(board->imu1), &(board->controls));
	//
	//if (control_input->run_mode==MOTORS_ON) {
		//// send values to servo outputs
		//servo_pwm_set(&(board->servos));
	//} else {	
		//servo_pwm_set(&(servo_failsafe));
	//}
	//
//
//}

task_return_t tasks_run_gps_update() {
	uint32_t tnow = time_keeper_get_millis();	
	
	gps_ublox_update();
	
 	//print_util_dbg_print("time :");
 	//print_util_dbg_print_num(tnow,10);
 	//print_util_dbg_print_num(board->GPS_data.timeLastMsg,10);
 	//print_util_dbg_print(" GPS status : 0x");
 	//print_util_dbg_print_num(board->GPS_data.status,16);
 	//print_util_dbg_print(" status:");
 	//print_util_dbg_print_num(board->GPS_data.accuracyStatus,10);
 	//print_util_dbg_print_num(board->GPS_data.horizontalStatus,10);
 	//print_util_dbg_print_num(board->GPS_data.altitudeStatus,10);
 	//print_util_dbg_print_num(board->GPS_data.speedStatus,10);
 	//print_util_dbg_print_num(board->GPS_data.courseStatus,10);
 	//print_util_dbg_print("\n");
	//
	/*if(gps_ublox_newValidGpsMsg())
	{
		print_util_dbg_print("GPS status:");
		print_util_dbg_print_num(board->GPS_data.status,10);
		print_util_dbg_print(" time gps:");
		print_util_dbg_print_num(board->GPS_data.timegps,10);
		print_util_dbg_print(" latitude :");
		print_util_dbg_print_num(board->GPS_data.latitude,10);
		print_util_dbg_print(" longitude :");
		print_util_dbg_print_num(board->GPS_data.longitude,10);
		print_util_dbg_print(" altitude");
		print_util_dbg_print_num(board->GPS_data.altitude,10);
		print_util_dbg_print("\n");
	}*/
}

task_return_t run_estimator()
{
	estimator_loop();
}

task_return_t tasks_run_navigation_update()
{
	int8_t i;
	
	if (((board->mav_state == MAV_STATE_ACTIVE)||(board->mav_state == MAV_STATE_CRITICAL))&&((board->mav_mode == MAV_MODE_AUTO_ARMED)||(board->mav_mode == MAV_MODE_GUIDED_ARMED)))
	{
		navigation_run();
	}
	
	
	//if ((board->number_of_waypoints > 0)&& waypoint_receiving == 0 )
	//{
		//print_util_dbg_print("List of Waypoint:");
		//for (i=0; i<board->number_of_waypoints; i++)
		//{
			//print_util_dbg_print("wp_id:");
			//print_util_dbg_print_num(board->waypoint_list[i].wp_id,10);
			//print_util_dbg_print(" autocontinue:");
			//print_util_dbg_print_num(board->waypoint_list[i].autocontinue,10);
			//print_util_dbg_print(" current:");
			//print_util_dbg_print_num(board->waypoint_list[i].current,10);
			//print_util_dbg_print(" frame:");
			//print_util_dbg_print_num(board->waypoint_list[i].frame,10);
			//print_util_dbg_print(" x:");
			//print_util_dbg_print_num(board->waypoint_list[i].x,10);
			//print_util_dbg_print(" y:");
			//print_util_dbg_print_num(board->waypoint_list[i].y,10);
			//print_util_dbg_print(" z:");
			//print_util_dbg_print_num(board->waypoint_list[i].z,10);
			//print_util_dbg_print(" params:");
			//print_util_dbg_print_num(board->waypoint_list[i].param1,10);
			//print_util_dbg_print_num(board->waypoint_list[i].param2,10);
			//print_util_dbg_print_num(board->waypoint_list[i].param3,10);
			//print_util_dbg_print_num(board->waypoint_list[i].param4,10);
			//print_util_dbg_print(";");
		//}
		//print_util_dbg_print("\n");
		//board->number_of_waypoints = 0;
	//}
	
}

task_return_t tasks_run_barometer_update()
{
	uint32_t tnow = time_keeper_get_micros();

	pressure_data *pressure = bmp085_get_pressure_data_slow(board->pressure.altitude_offset);
	board->pressure =  *pressure;
	
	
}

task_return_t send_rt_stats() {
	
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, time_keeper_get_millis(), "stabAvgDelay", main_tasks.tasks[1].delay_avg);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, time_keeper_get_millis(), "stabDelayVar", sqrt(main_tasks.tasks[1].delay_var_squared));
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, time_keeper_get_millis(), "stabMaxDelay", main_tasks.tasks[1].delay_max);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, time_keeper_get_millis(), "stabRTvio", main_tasks.tasks[0].rt_violations);

	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, time_keeper_get_millis(), "imuExTime", main_tasks.tasks[0].execution_time);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, time_keeper_get_millis(), "stabExTime", main_tasks.tasks[1].execution_time);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, time_keeper_get_millis(), "estExTime", main_tasks.tasks[3].execution_time);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, time_keeper_get_millis(), "MVLExTime", main_tasks.tasks[9].execution_time);

	
	main_tasks.tasks[1].rt_violations=0;
	main_tasks.tasks[1].delay_max=0;

}


void create_tasks() {
	scheduler_init(&main_tasks);
	board=get_board_hardware();
	
	//	scheduler_register_task(&main_tasks, 0, 4000, RUN_REGULAR, &tasks_run_imu_update );
	scheduler_register_task(&main_tasks, 1, 4000, RUN_REGULAR, &tasks_run_stabilisation );
	scheduler_register_task(&main_tasks, 2, 1000, RUN_REGULAR, &mavlink_protocol_update);
	
	scheduler_register_task(&main_tasks, 3, 100000, RUN_REGULAR, &tasks_run_gps_update);
	//scheduler_register_task(&main_tasks, 4, 4000, RUN_REGULAR, &run_estimator);
	//scheduler_register_task(&main_tasks, 4, 100000, RUN_REGULAR, &radar_module_read);

	scheduler_register_task(&main_tasks, 5, 10000, RUN_REGULAR, &tasks_run_navigation_update);

	scheduler_register_task(&main_tasks, 6, 250000, RUN_REGULAR, &tasks_set_mav_mode_n_state);
	
	scheduler_register_task(&main_tasks, 7, 150000, RUN_REGULAR, &tasks_run_barometer_update);

	scheduler_add_task(mavlink_stream_get_taskset(),  1000000, RUN_NEVER, &send_rt_stats, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT);
}