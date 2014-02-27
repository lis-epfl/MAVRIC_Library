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
#include "led.h"
#include "imu.h"
#include "orca.h"
#include "delay.h"

NEW_TASK_SET(main_tasks, 10)

#define PRESSURE_LPF 0.1

central_data_t *centralData;

//bool has_started_engines;

task_set* get_main_taskset() {
	return &main_tasks;
}


void rc_user_channels(uint8_t *chanSwitch, int8_t *rc_check, int8_t *motorbool)
{
	
	get_channel_mode(chanSwitch);
	
	if ((rc_get_channel_neutral(RC_TRIM_P3) * RC_SCALEFACTOR)>0.0)
	{
		centralData->collision_avoidance = true;
	}else{
		centralData->collision_avoidance = false;
	}
	
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0,get_millis(),"P1",rc_get_channel_neutral(RC_TRIM_P1));
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0,get_millis(),"P2",rc_get_channel_neutral(RC_TRIM_P2));
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0,get_millis(),"P3",rc_get_channel_neutral(RC_TRIM_P3));
	
	//dbg_print("chanSwitch ");
	//dbg_print_num(*chanSwitch,10);
	//dbg_print_num(getChannel(4),10);
	//dbg_print_num(getChannel(5),10);
	//dbg_print("\n");
	
	if((get_thrust_from_remote()<-0.95) && (get_yaw_from_remote() > 0.9))
	{
		//dbg_print("motor on\n");
		//dbg_print("motor on: yaw=\n"); dbg_putfloat(get_yaw_from_remote(),2);
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

void switch_off_motors()
{
	dbg_print("Switching off motors!\n");
	centralData->run_mode = MOTORS_OFF;
	//has_started_engines = false;
	centralData->mav_state = MAV_STATE_STANDBY;
	centralData->mav_mode = MAV_MODE_MANUAL_DISARMED;
	
	centralData->in_the_air = false;
}

void relevel_imu()
{
	uint8_t i;
	centralData->imu1.attitude.calibration_level=LEVELING;	
	centralData->mav_state = MAV_STATE_CALIBRATING;
	centralData->mav_mode = MAV_MODE_PREFLIGHT;

	dbg_print("calibrating IMU...\n");
	//calibrate_Gyros(&centralData->imu1);
	for (i=1000; i>0; i--) {
		run_imu_update();
		mavlink_protocol_update();	
		delay_ms(5);
	}
	// after initial leveling, initialise accelerometer biases
	
	/*
	centralData->imu1.attitude.calibration_level=LEVEL_PLUS_ACCEL;
	for (i=100; i>0; i--) {
		imu_update(&(centralData->imu1), &centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);	
		mavlink_protocol_update();			
		delay_ms(5);
	}*/
	
	for (i=0;i<3;i++)
	{
		centralData->position_estimator.vel_bf[i]=0.0;
		centralData->position_estimator.vel[i]=0.0;
	}
	
	centralData->imu1.attitude.calibration_level=OFF;
	
	centralData->mav_state = MAV_STATE_STANDBY;
	centralData->mav_mode = MAV_MODE_MANUAL_DISARMED;
}

task_return_t set_mav_mode_n_state()
{
	uint8_t channelSwitches = 0;
	int8_t RC_check = 0;
	int8_t motor_switch = 0;
	
	float distFromHomeSqr;
	
	LED_Toggle(LED1);
	
	rc_user_channels(&channelSwitches,&RC_check, &motor_switch);
	
	switch(centralData->mav_state)
	{
		case MAV_STATE_BOOT:
			break;
		case MAV_STATE_CALIBRATING:
			break;
		case MAV_STATE_STANDBY:
			if (motor_switch == 1)
			{
				switch(channelSwitches)
				{
					case 0:
						dbg_print("Switching on the motors!\n");
						position_reset_home_altitude(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data,&centralData->sim_model.localPosition);
						centralData->waypoint_set = false;
						centralData->run_mode = MOTORS_ON;
						//has_started_engines = true;
						//centralData->mav_state = MAV_STATE_ACTIVE;
						centralData->mav_mode = MAV_MODE_MANUAL_ARMED;
						break;
					case 1:
						dbg_print("Switches not ready, both should be pushed!\n");
						break;
					case 2:
						dbg_print("Switches not ready, both should be pushed!\n");
						break;
					case 3:
						dbg_print("Switches not ready, both should be pushed!\n");
						break;
				}
			}
			if (centralData->run_mode == MOTORS_ON)
			//if (has_started_engines)
			{
				switch (channelSwitches)
				{
					case 0:
						centralData->mav_mode = MAV_MODE_MANUAL_ARMED;
						break;
					case 1:
						centralData->mav_mode = MAV_MODE_STABILIZE_ARMED;
						break;
					case 2:
						if (centralData->in_the_air)
						{
							centralData->mav_mode = MAV_MODE_GUIDED_ARMED;
							// Automatic take-off mode
							if (centralData->mav_mode_previous != MAV_MODE_GUIDED_ARMED)
							{
								centralData->automatic_take_off = true;
							}
						}
						break;
					case 3:
						if (centralData->in_the_air)
						{
							//centralData->mav_state = MAV_STATE_ACTIVE;
							centralData->mav_mode = MAV_MODE_AUTO_ARMED;
							
							// Automatic take-off mode
							if (centralData->mav_mode_previous != MAV_MODE_AUTO_ARMED)
							{
								centralData->automatic_take_off = true;
							}
						}
						break;
				}
				
				switch (centralData->mav_mode)
				{
					case MAV_MODE_MANUAL_ARMED:
						if (centralData->in_the_air)
						{
							centralData->mav_state = MAV_STATE_ACTIVE;
						}
						break;
					case MAV_MODE_STABILIZE_ARMED:
						if (centralData->in_the_air)
						{
							centralData->mav_state = MAV_STATE_ACTIVE;
						}
						break;
					case MAV_MODE_GUIDED_ARMED:
						// Automatic take-off mode
						if(centralData->automatic_take_off)
						{
							centralData->automatic_take_off = false;
							wp_take_off();
						}
						
						distFromHomeSqr = SQR(centralData->position_estimator.localPosition.pos[X]-centralData->waypoint_hold_coordinates.pos[X]) + SQR(centralData->position_estimator.localPosition.pos[Y]-centralData->waypoint_hold_coordinates.pos[Y]) + SQR(centralData->position_estimator.localPosition.pos[Z]-centralData->waypoint_hold_coordinates.pos[Z]);
						if ((centralData->dist2wp_sqr <= 16.0)&&(!centralData->automatic_take_off))
						{
							centralData->mav_state = MAV_STATE_ACTIVE;
							dbg_print("Automatic take-off finised, distFromHomeSqr (10x):");
							dbg_print_num(distFromHomeSqr*10.0,10);
						}
						break;
					case MAV_MODE_AUTO_ARMED:
						if(centralData->automatic_take_off)
						{
							centralData->automatic_take_off = false;
							wp_take_off();
						}
						//if (centralData->mav_mode_previous != MAV_MODE_AUTO_ARMED)
						//{
							//wp_hold_init();
						//}
						if (!centralData->waypoint_set)
						{
							init_wp();
						}
						distFromHomeSqr = SQR(centralData->position_estimator.localPosition.pos[X]-centralData->waypoint_hold_coordinates.pos[X]) + SQR(centralData->position_estimator.localPosition.pos[Y]-centralData->waypoint_hold_coordinates.pos[Y]) + SQR(centralData->position_estimator.localPosition.pos[Z]-centralData->waypoint_hold_coordinates.pos[Z]);
						if ((centralData->dist2wp_sqr <= 16.0)&&(!centralData->automatic_take_off))
						{
							centralData->mav_state = MAV_STATE_ACTIVE;
							dbg_print("Automatic take-off finised, distFromHomeSqr (10x):");
							dbg_print_num(distFromHomeSqr*10.0,10);
						}
						break;
				}
				switch (RC_check)
				{
					case 1:
					break;
					case -1:
					centralData->mav_state = MAV_STATE_CRITICAL;
					break;
					case -2:
					centralData->mav_state = MAV_STATE_CRITICAL;
					break;
				}
				if (get_thrust_from_remote()>-0.7)
				{
					centralData->in_the_air = true;
				}
			}
			if (motor_switch == -1)
			{
				switch_off_motors();
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
			
			switch (centralData->mav_mode)
			{
				case MAV_MODE_MANUAL_ARMED:
					break;
				case MAV_MODE_STABILIZE_ARMED:
					break;
				case MAV_MODE_GUIDED_ARMED:
					if (centralData->mav_mode_previous != MAV_MODE_GUIDED_ARMED)
					{
						wp_hold_init(centralData->position_estimator.localPosition);
					}
					break;
				case MAV_MODE_AUTO_ARMED:
					if (centralData->mav_mode_previous != MAV_MODE_AUTO_ARMED)
					{
						wp_hold_init(centralData->position_estimator.localPosition);
					}
					if (!centralData->waypoint_set)
					{
						init_wp();
					}
					waypoint_navigation_handler();
					break;
			}
			
			//dbg_print("motor_switch: ");
			//dbg_print_num(motor_switch,10);
			if (motor_switch == -1)
			{
				switch_off_motors();
			}
		
			switch (RC_check)
			{
				case 1:
					break;
				case -1:
					centralData->mav_state = MAV_STATE_CRITICAL;
					break;
				case -2:
					centralData->mav_state = MAV_STATE_CRITICAL;
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
					centralData->mav_mode = MAV_MODE_AUTO_ARMED;
					break;
			}
			if (motor_switch == -1)
			{
				switch_off_motors();
			}
			
			switch (centralData->mav_mode)
			{
				case MAV_MODE_GUIDED_ARMED:
				case MAV_MODE_AUTO_ARMED:
					if (centralData->mav_state_previous != MAV_STATE_CRITICAL)
					{
						centralData->critical_behavior = CLIMB_TO_SAFE_ALT;
						centralData->critical_next_state = false;
					}
					waypoint_critical_handler();
					break;
			}
			
			switch (RC_check)
			{
				case 1:  // !! only if receivers are back, switch into appropriate mode
					centralData->mav_state = MAV_STATE_ACTIVE;
					centralData->critical_behavior = CLIMB_TO_SAFE_ALT;
					centralData->critical_next_state = false;
					break;
				case -1:
					break;
				case -2:
					//if (centralData->critical_landing)
					//{
						centralData->mav_state = MAV_STATE_EMERGENCY;
					//}
					//centralData->mav_state = MAV_STATE_EMERGENCY;
					break;
			}
			break;
		case MAV_STATE_EMERGENCY:
			//if (centralData->position_estimator.localPosition.pos[Z] < 1.0)
			{
				centralData->mav_mode = MAV_MODE_MANUAL_DISARMED;
				centralData->controls.control_mode = ATTITUDE_COMMAND_MODE;
				switch (RC_check)
				{
					case 1:
						centralData->mav_state = MAV_STATE_STANDBY;
						break;
					case -1:
						break;
					case -2:
						break;
				}
			}
			break;
	}

	//dbg_print("MAV state :");
	//dbg_print_num(centralData->mav_state,10);
	//dbg_print(", MAV mode :");
	//dbg_print_num(centralData->mav_mode,10);
	//dbg_print("\n");
	
	centralData->mav_mode_previous = centralData->mav_mode;
	centralData->mav_state_previous = centralData->mav_state;
	
	if (centralData->simulation_mode_previous != centralData->simulation_mode)
	{
		uint8_t i;
		// From simulation to reality
		if (centralData->simulation_mode == 0)
		{
			centralData->position_estimator.localPosition.origin = centralData->sim_model.localPosition.origin;
			for (i=0;i<3;i++)
			{
				centralData->position_estimator.localPosition.pos[i] = centralData->sim_model.localPosition.pos[i];
			}
			centralData->position_estimator.init_gps_position = false;
			centralData->mav_state = MAV_STATE_STANDBY;
			centralData->mav_mode = MAV_MODE_MANUAL_DISARMED;
			//relevel_imu();
		}
		// From reality to simulation
		if (centralData->simulation_mode == 1)
		{
			//centralData->sim_model.localPosition.origin = centralData->position_estimator.localPosition.origin;
			//for (i=0;i<3;i++)
			//{
				//centralData->sim_model.localPosition.pos[i] = centralData->position_estimator.localPosition.pos[i];
			//}
			//centralData->sim_model.attitude = centralData->imu1.attitude;
			
			
			init_simulation(&(centralData->sim_model),&(centralData->imu1.attitude),centralData->position_estimator.localPosition);
			centralData->position_estimator.init_gps_position = false;
			
		}
	}
	
	centralData->simulation_mode_previous = centralData->simulation_mode;
	
	//mavlink_msg_named_value_int_send(MAVLINK_COMM_0,get_millis(),"run_mode", centralData->run_mode);
	//dbg_print_num(centralData->run_mode,10);
	//mavlink_msg_named_value_int_send(MAVLINK_COMM_0,get_millis(),"in_the_air", centralData->in_the_air);
}

void run_imu_update() {
	if (centralData->simulation_mode==1) {
		simu_update(&centralData->sim_model, &centralData->servos, &(centralData->imu1), &centralData->position_estimator);
		imu_update(&(centralData->imu1), &(centralData->position_estimator), &centralData->pressure, &centralData->GPS_data);
		
	} else {
		imu_get_raw_data(&(centralData->imu1));
		imu_update(&(centralData->imu1), &centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);
	}
}	


task_return_t run_stabilisation() {
	int i;
	
	run_imu_update();

	switch(centralData->mav_mode)
	{
		
		case MAV_MODE_MANUAL_ARMED:
			centralData->controls = get_command_from_remote();
			centralData->controls.control_mode = ATTITUDE_COMMAND_MODE;
			centralData->controls.yaw_mode=YAW_RELATIVE;
			
			cascade_stabilise_copter(&(centralData->imu1), &centralData->position_estimator, &(centralData->controls));
			break;
		case MAV_MODE_STABILIZE_ARMED:
			centralData->controls = get_command_from_remote();
			centralData->controls.control_mode = VELOCITY_COMMAND_MODE;
			centralData->controls.yaw_mode=YAW_RELATIVE;
			
			//centralData->controls.tvel[X]=-10.0*centralData->controls.rpy[PITCH];
			//centralData->controls.tvel[Y]= 10.0*centralData->controls.rpy[ROLL];
			//centralData->controls.tvel[Z]=- 1.5*centralData->controls.thrust;
			get_velocity_vector_from_remote(centralData->controls.tvel);
			
			cascade_stabilise_copter(&(centralData->imu1), &centralData->position_estimator, &(centralData->controls));
			
			break;
		case MAV_MODE_GUIDED_ARMED:
			centralData->controls = centralData->controls_nav;
			centralData->controls.control_mode = VELOCITY_COMMAND_MODE;
			centralData->controls.yaw_mode = YAW_ABSOLUTE;
			
			cascade_stabilise_copter(&(centralData->imu1), &centralData->position_estimator, &(centralData->controls));
			break;
		case MAV_MODE_AUTO_ARMED:
			centralData->controls = centralData->controls_nav;
			centralData->controls.control_mode = VELOCITY_COMMAND_MODE;
			
			// if no waypoints are set, we do position hold therefore the yaw mode is absolute
			if(centralData->waypoint_set&&(centralData->mav_state!=MAV_STATE_STANDBY))
			{
				centralData->controls.yaw_mode = YAW_COORDINATED;
			}else{
				centralData->controls.yaw_mode = YAW_ABSOLUTE;
			}
			
			
			cascade_stabilise_copter(&(centralData->imu1), &centralData->position_estimator, &(centralData->controls));
			break;
		
		case MAV_MODE_PREFLIGHT:
		case MAV_MODE_MANUAL_DISARMED:
		case MAV_MODE_STABILIZE_DISARMED:
		case MAV_MODE_GUIDED_DISARMED:
		case MAV_MODE_AUTO_DISARMED:
			centralData->run_mode = MOTORS_OFF;
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

void fake_gps_fix()
{
	local_coordinates_t fake_pos;
	
	fake_pos.pos[X] = 10.0;
	fake_pos.pos[Y] = 10.0;
	fake_pos.pos[Z] = 0.0;
	fake_pos.origin.latitude = HOME_LATITUDE;
	fake_pos.origin.longitude = HOME_LONGITUDE;
	fake_pos.origin.altitude = HOME_ALTITUDE;
	fake_pos.timestamp_ms = centralData->position_estimator.localPosition.timestamp_ms;
	global_position_t gpos = local_to_global_position(fake_pos);
	
	centralData->GPS_data.latitude = gpos.latitude;
	centralData->GPS_data.longitude = gpos.longitude;
	centralData->GPS_data.altitude = gpos.altitude;
	
	centralData->GPS_data.timeLastMsg = get_millis();
	
	centralData->GPS_data.status = GPS_OK;
	
}

task_return_t gps_task() {
	uint32_t tnow = get_millis();	
	if (centralData->simulation_mode==1) {
		simulate_gps(&centralData->sim_model, &centralData->GPS_data);
	} else {
		gps_update();
		//fake_gps_fix();
	}
}

task_return_t run_navigation_task()
{
	int8_t i;
	
		switch (centralData->mav_state)
		{
			case MAV_STATE_STANDBY:
				if (((centralData->mav_mode == MAV_MODE_GUIDED_ARMED)||(centralData->mav_mode == MAV_MODE_AUTO_ARMED)) && !centralData->automatic_take_off)
				{
					run_navigation(centralData->waypoint_hold_coordinates);
				}
				break;
			case MAV_STATE_ACTIVE:
				switch (centralData->mav_mode)
				{
					case MAV_MODE_AUTO_ARMED:
						if (centralData->waypoint_set)
						{
							run_navigation(centralData->waypoint_coordinates);
						}else{
							run_navigation(centralData->waypoint_hold_coordinates);
						}
						//run_navigation(centralData->waypoint_coordinates);
						break;
					case MAV_MODE_GUIDED_ARMED:
						run_navigation(centralData->waypoint_hold_coordinates);
						break;
				}
				break;
			case MAV_STATE_CRITICAL:
				if ((centralData->mav_mode == MAV_MODE_GUIDED_ARMED)||(centralData->mav_mode == MAV_MODE_AUTO_ARMED))
				{
					run_navigation(centralData->waypoint_critical_coordinates);
				}
				break;
		}
	
}
uint32_t last_baro_update;
task_return_t run_barometer()
{
	uint32_t tnow = get_micros();
	central_data_t *central_data=get_central_data();
	
	pressure_data *pressure= get_pressure_data_slow(centralData->pressure.altitude_offset);
	if (central_data->simulation_mode==1) {
		simulate_barometer(&centralData->sim_model, pressure);
	} 
	centralData->pressure=*pressure;
	
}


void create_tasks() {
	
	//has_started_engines = false;
	
	init_scheduler(&main_tasks);
	
	centralData = get_central_data();
	
	register_task(&main_tasks, 0, 4000, RUN_REGULAR, &run_stabilisation );
	
	register_task(&main_tasks, 1, 15000, RUN_REGULAR, &run_barometer);
	main_tasks.tasks[1].timing_mode=PERIODIC_RELATIVE;

	register_task(&main_tasks, 2, 100000, RUN_REGULAR, &gps_task);
	//register_task(&main_tasks, 4, 4000, RUN_REGULAR, &run_estimator);
	//register_task(&main_tasks, , 100000, RUN_REGULAR, &read_radar);

	register_task(&main_tasks, 3, ORCA_TIME_STEP_MILLIS * 1000.0, RUN_REGULAR, &run_navigation_task);

	register_task(&main_tasks, 4, 200000, RUN_REGULAR, &set_mav_mode_n_state);
	

	register_task(&main_tasks, 5, 4000, RUN_REGULAR, &mavlink_protocol_update);

}