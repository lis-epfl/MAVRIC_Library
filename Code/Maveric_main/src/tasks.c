/*
 * tasks.c
 *
 * Created: 16/09/2013 13:28:11
 *  Author: sfx
 */ 


#include "tasks.h"
#include "central_data.h"
#include "print_util.h"
#include "stabilisation.h"
#include "gps_ublox.h"
#include "estimator.h"
#include "navigation.h"


NEW_TASK_SET(main_tasks, 10)

#define PRESSURE_LPF 0.1

central_data_t *centralData;

task_set* get_main_taskset() {
	return &main_tasks;
}

task_return_t run_imu_update() {
	imu_update(&(centralData->imu1));	
}	

void rc_user_channels(uint8_t *chanSwitch, int8_t *rc_check, int8_t *motorbool)
{
	
	get_channel_mode(chanSwitch);
	
	//dbg_print("chanSwitch ");
	//dbg_print_num(*chanSwitch,10);
	//dbg_print_num(getChannel(4),10);
	//dbg_print_num(getChannel(5),10);
	//dbg_print("\n");
	
	if((get_thrust_from_remote()<-0.95) && (get_yaw_from_remote() > 0.9))
	{
		//dbg_print("motor on\n");
		*motorbool = 1;
	}else if((get_thrust_from_remote()<-0.95) && (get_yaw_from_remote() <-0.9))
	{
		//dbg_print("motor off\n");
		*motorbool = -1;
	}else{
		//dbg_print("motor nothing\n");
		*motorbool = 0;
	}
	
	switch (rc_check_receivers())
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
	//dbg_print("rc_check: ");
	//dbg_print_num(rc_check,10);
	//dbg_print("; motorbool : ");
	//dbg_print_num(*motorbool,10);
	//dbg_print("\n");
}

task_return_t set_mav_mode_n_state()
{
	uint8_t channelSwitches = 0;
	int8_t RC_check = 0;
	int8_t motor_switch = 0;
	
	rc_user_channels(&channelSwitches,&RC_check, &motor_switch);
	
	switch(centralData->mav_state)
	{
		case MAV_STATE_CALIBRATING:
			break;
		case MAV_STATE_STANDBY:
			if (motor_switch == 1)
			{
				switch(channelSwitches)
				{
					case 0:
						dbg_print("Switching on the motors!\n");
						centralData->controls.run_mode = MOTORS_ON;
						centralData->mav_state = MAV_STATE_ACTIVE;
						centralData->mav_mode = MAV_MODE_MANUAL_ARMED;
						break;
					case 1:
						dbg_print("Switches not ready, both should be pushed!\n");
						//centralData->controls.run_mode = MOTORS_ON;
						//centralData->mav_state = MAV_STATE_ACTIVE;
						//centralData->mav_mode = MAV_MODE_STABILIZE_ARMED;
						break;
					case 2:
						dbg_print("Switches not ready, both should be pushed!\n");
						break;
					case 3:
						dbg_print("Switches not ready, both should be pushed!\n");
						break;
				}
			}
			break;
		case MAV_STATE_ACTIVE:
			switch(channelSwitches)
			{
				case 0:
					centralData->mav_mode= MAV_MODE_MANUAL_ARMED;
					break;
				case 1:
					centralData->mav_mode= MAV_MODE_STABILIZE_ARMED;
					break;
				case 2:
					centralData->mav_mode = MAV_MODE_GUIDED_ARMED;
					break;
				case 3:
					centralData->mav_mode = MAV_MODE_AUTO_ARMED;
					break;
			}
			
			
			//dbg_print("motor_switch: ");
			//dbg_print_num(motor_switch,10);
			if (motor_switch == -1)
			{
				dbg_print("Switching off motors!\n");
				centralData->controls.run_mode = MOTORS_OFF;
				centralData->mav_state = MAV_STATE_STANDBY;
				centralData->mav_mode = MAV_MODE_MANUAL_DISARMED;
			}
		
			switch (RC_check)
			{
				case 1:
					break;
				case -1:
					centralData->mav_state = MAV_STATE_CRITICAL;
					break;
				case -2:
					centralData->mav_state = MAV_STATE_EMERGENCY;
					break;
			}
			break;
		case MAV_STATE_CRITICAL:
			switch(channelSwitches)
			{
				case 0:
					centralData->mav_mode= MAV_MODE_MANUAL_ARMED;
					break;
				case 1:
					centralData->mav_mode= MAV_MODE_STABILIZE_ARMED;
					break;
				case 2:
					centralData->mav_mode = MAV_MODE_GUIDED_ARMED;
					break;
				case 3:
					break;
			}
			if (motor_switch == -1)
			{
				dbg_print("Switching off motors!\n");
				centralData->controls.run_mode = MOTORS_OFF;
				centralData->mav_state = MAV_STATE_STANDBY;
				centralData->mav_mode = MAV_MODE_MANUAL_DISARMED;
			}
			switch (RC_check)
			{
				case 1:
					centralData->mav_state = MAV_STATE_ACTIVE;
					break;
				case -1:
					break;
				case -2:
					centralData->mav_state = MAV_STATE_EMERGENCY;
					break;
			}
			break;
		case MAV_STATE_EMERGENCY:
			centralData->mav_mode = MAV_MODE_MANUAL_DISARMED;
			centralData->controls.control_mode = ATTITUDE_COMMAND_MODE_REL_YAW;
			if (centralData->imu1.attitude.localPosition.pos[Z] < 0.5)
			{
				centralData->mav_state = MAV_STATE_STANDBY;
			}
			break;
	}

	//dbg_print("MAV state :");
	//dbg_print_num(centralData->mav_state,10);
	//dbg_print(", MAV mode :");
	//dbg_print_num(centralData->mav_mode,10);
	//dbg_print("\n");
	
}

task_return_t run_stabilisation() {
	int i;
	
	if (centralData->simulation_mode==1) {
		simu_update(&(centralData->sim_model), &(centralData->servos), &(centralData->imu1));
	} else {
		imu_update(&(centralData->imu1));
	}

	switch(centralData->mav_mode)
	{
		
		case MAV_MODE_MANUAL_ARMED:
			centralData->waypoint_hold_init = false;
			centralData->mission_started = false;
			centralData->controls = get_command_from_remote();
			
			centralData->controls.control_mode = ATTITUDE_COMMAND_MODE_REL_YAW;
			
			quad_stabilise(&(centralData->imu1), &(centralData->controls));
			break;
		case MAV_MODE_STABILIZE_ARMED:
			centralData->waypoint_hold_init = false;
			centralData->mission_started = false;
			centralData->controls = get_command_from_remote();
			//dbg_print("Thrust:");
			//dbg_print_num(centralData->controls.thrust*10000,10);
			//dbg_print("\n");
			centralData->controls.control_mode = VELOCITY_COMMAND_MODE;
			
			centralData->controls.tvel[X]=-10.0*centralData->controls.rpy[PITCH];
			centralData->controls.tvel[Y]= 10.0*centralData->controls.rpy[ROLL];
			centralData->controls.tvel[Z]=- 3.0*centralData->controls.thrust;
			
			quad_stabilise(&(centralData->imu1), &(centralData->controls));
			
			break;
		case MAV_MODE_GUIDED_ARMED:
			centralData->mission_started = false;
			centralData->controls = centralData->controls_nav;
			//centralData->controls.thrust = f_min(get_thrust_from_remote()*100000.0,centralData->controls_nav.thrust*100000.0)/100000.0;
			//centralData->controls.thrust = get_thrust_from_remote();
			
			//dbg_print("Thrust (x10000):");
			//dbg_print_num(centralData->controls.thrust*10000.0,10);
			//dbg_print(", remote (x10000):");
			//dbg_print_num(get_thrust_from_remote()*10000.0,10);
			//dbg_print(" => min (x10000):");
			//dbg_print_num(f_min(get_thrust_from_remote()*100000.0,centralData->controls_nav.thrust*100000.0)/100000.0 *10000.0,10);
			//dbg_print("\n");
			
			quad_stabilise(&(centralData->imu1), &(centralData->controls));
			break;
		case MAV_MODE_AUTO_ARMED:
			centralData->mission_started = true;
			centralData->waypoint_hold_init = false;
			centralData->controls = centralData->controls_nav;
			//centralData->controls.thrust = f_min(get_thrust_from_remote()*100000.0,centralData->controls_nav.thrust*100000.0)/100000.0;
			//centralData->controls.thrust = get_thrust_from_remote();
			
			//dbg_print("Thrust main:");
			//dbg_print_num(centralData->controls.thrust*10000,10);
			//dbg_print("\n");
			
			quad_stabilise(&(centralData->imu1), &(centralData->controls));
			break;
		
		case MAV_MODE_PREFLIGHT:
		case MAV_MODE_MANUAL_DISARMED:
		case MAV_MODE_STABILIZE_DISARMED:
		case MAV_MODE_GUIDED_DISARMED:
		case MAV_MODE_AUTO_DISARMED:
			//set_servos(&(servo_failsafe));
			for (i=0; i<NUMBER_OF_SERVO_OUTPUTS; i++) {
				centralData->servos[i]=servo_failsafe[i];
			}
			break;
		
	}
	
	// !!! -- for safety, this should remain the only place where values are written to the servo outputs! --- !!!
	if (centralData->simulation_mode!=1) {
		set_servos(&(centralData->servos));
	}
		

}

task_return_t gps_task() {
	uint32_t tnow = get_millis();	
	
	gps_update();
	
 	//dbg_print("time :");
 	//dbg_print_num(tnow,10);
 	//dbg_print_num(centralData->GPS_data.timeLastMsg,10);
 	//dbg_print(" GPS status : 0x");
 	//dbg_print_num(centralData->GPS_data.status,16);
 	//dbg_print(" status:");
 	//dbg_print_num(centralData->GPS_data.accuracyStatus,10);
 	//dbg_print_num(centralData->GPS_data.horizontalStatus,10);
 	//dbg_print_num(centralData->GPS_data.altitudeStatus,10);
 	//dbg_print_num(centralData->GPS_data.speedStatus,10);
 	//dbg_print_num(centralData->GPS_data.courseStatus,10);
 	//dbg_print("\n");
	//
	/*if(newValidGpsMsg())
	{
		dbg_print("GPS status:");
		dbg_print_num(centralData->GPS_data.status,10);
		dbg_print(" time gps:");
		dbg_print_num(centralData->GPS_data.timegps,10);
		dbg_print(" latitude :");
		dbg_print_num(centralData->GPS_data.latitude,10);
		dbg_print(" longitude :");
		dbg_print_num(centralData->GPS_data.longitude,10);
		dbg_print(" altitude");
		dbg_print_num(centralData->GPS_data.altitude,10);
		dbg_print("\n");
	}*/
}

task_return_t run_estimator()
{
	estimator_loop();
}

task_return_t run_navigation_task()
{
	int8_t i;
	
	if (((centralData->mav_state == MAV_STATE_ACTIVE)||(centralData->mav_state == MAV_STATE_CRITICAL))&&((centralData->mav_mode == MAV_MODE_AUTO_ARMED)||(centralData->mav_mode == MAV_MODE_GUIDED_ARMED)))
	{
		run_navigation();
	}
	
	
	//if ((centralData->number_of_waypoints > 0)&& waypoint_receiving == 0 )
	//{
		//dbg_print("List of Waypoint:");
		//for (i=0; i<centralData->number_of_waypoints; i++)
		//{
			//dbg_print("wp_id:");
			//dbg_print_num(centralData->waypoint_list[i].wp_id,10);
			//dbg_print(" autocontinue:");
			//dbg_print_num(centralData->waypoint_list[i].autocontinue,10);
			//dbg_print(" current:");
			//dbg_print_num(centralData->waypoint_list[i].current,10);
			//dbg_print(" frame:");
			//dbg_print_num(centralData->waypoint_list[i].frame,10);
			//dbg_print(" x:");
			//dbg_print_num(centralData->waypoint_list[i].x,10);
			//dbg_print(" y:");
			//dbg_print_num(centralData->waypoint_list[i].y,10);
			//dbg_print(" z:");
			//dbg_print_num(centralData->waypoint_list[i].z,10);
			//dbg_print(" params:");
			//dbg_print_num(centralData->waypoint_list[i].param1,10);
			//dbg_print_num(centralData->waypoint_list[i].param2,10);
			//dbg_print_num(centralData->waypoint_list[i].param3,10);
			//dbg_print_num(centralData->waypoint_list[i].param4,10);
			//dbg_print(";");
		//}
		//dbg_print("\n");
		//centralData->number_of_waypoints = 0;
	//}
	
}

task_return_t run_barometer()
{
	uint32_t tnow = get_micros();

	pressure_data *pressure = get_pressure_data_slow(centralData->pressure.altitude_offset);
	centralData->pressure =  *pressure;
	
	
}

task_return_t send_rt_stats() {
	
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "stabAvgDelay", main_tasks.tasks[1].delay_avg);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "stabDelayVar", sqrt(main_tasks.tasks[1].delay_var_squared));
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "stabMaxDelay", main_tasks.tasks[1].delay_max);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "stabRTvio", main_tasks.tasks[0].rt_violations);

	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "imuExTime", main_tasks.tasks[0].execution_time);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "stabExTime", main_tasks.tasks[1].execution_time);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "navExTime", main_tasks.tasks[3].execution_time);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "imu_dt", get_central_data()->imu1.dt);

	
	main_tasks.tasks[1].rt_violations=0;
	main_tasks.tasks[1].delay_max=0;

}


void create_tasks() {
	init_scheduler(&main_tasks);
	//board=get_board_hardware();
	centralData = get_central_data();
	
	//	register_task(&main_tasks, 0, 4000, RUN_REGULAR, &run_imu_update );
	register_task(&main_tasks, 1, 4000, RUN_REGULAR, &run_stabilisation );
	register_task(&main_tasks, 2, 1000, RUN_REGULAR, &mavlink_protocol_update);
	
	register_task(&main_tasks, 3, 100000, RUN_REGULAR, &gps_task);
	//register_task(&main_tasks, 4, 4000, RUN_REGULAR, &run_estimator);
	//register_task(&main_tasks, 4, 100000, RUN_REGULAR, &read_radar);

	register_task(&main_tasks, 5, 10000, RUN_REGULAR, &run_navigation_task);

	register_task(&main_tasks, 6, 1000000, RUN_REGULAR, &set_mav_mode_n_state);
	
	register_task(&main_tasks, 7, 150000, RUN_REGULAR, &run_barometer);

	add_task(get_mavlink_taskset(),  1000000, RUN_NEVER, &send_rt_stats, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT);
}