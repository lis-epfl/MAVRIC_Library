/*
 * mavlink_stream.c
 *
 * a wrapper for mavlink to use the stream interface
 *
 * Created: 14/02/2013 17:10:05
 *  Author: Felix, Julien, Nicolas
 */ 

#include "mavlink_stream.h"
#include "buffer.h"
#include "onboard_parameters.h"
#include "print_util.h"
#include "central_data.h"
#include "mavlink_waypoint_handler.h"
#include "neighbor_selection.h"

byte_stream_t* mavlink_out_stream;
byte_stream_t* mavlink_in_stream;
Buffer_t mavlink_in_buffer;

central_data_t *centralData;

NEW_TASK_SET (mavlink_tasks, 30)

void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
	if (chan == MAVLINK_COMM_0)
	{
		//uart0_transmit(ch);
		mavlink_out_stream->put(mavlink_out_stream->data, ch);
	}
	if (chan == MAVLINK_COMM_1)
	{
		//uart1_transmit(ch);
	}
}

void mavlink_receive_handler() {
	Mavlink_Received_t rec;
	
	if(mavlink_receive(mavlink_in_stream, &rec)) {
		
		if (rec.msg.sysid == MAVLINK_BASE_STATION_ID)
		{
			dbg_print("\n Received message with ID");
			dbg_print_num(rec.msg.msgid, 10);
			dbg_print(" from system");
			dbg_print_num(rec.msg.sysid, 10);
			dbg_print(" for component");
			dbg_print_num(rec.msg.compid,10);
			dbg_print( "\n");
			
			handle_mavlink_message(&rec);
		}else if (rec.msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
		{
			//dbg_print("\n Received message with ID");
			//dbg_print_num(rec.msg.msgid, 10);
			//dbg_print(" from system");
			//dbg_print_num(rec.msg.sysid, 10);
			//dbg_print(" for component");
			//dbg_print_num(rec.msg.compid,10);
			//dbg_print( "\n");
			
			read_msg_from_neighbors(&rec);
		}
		
	}
}

void init_mavlink(byte_stream_t *transmit_stream, byte_stream_t *receive_stream, int sysid) {
	mavlink_system.sysid = sysid; // System ID, 1-255
	mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
	mavlink_system.type = MAV_TYPE_QUADROTOR;
	
	mavlink_mission_planner.sysid = mavlink_system.sysid;
	mavlink_mission_planner.compid = MAV_COMP_ID_MISSIONPLANNER;
	mavlink_mission_planner.type = MAV_TYPE_QUADROTOR;
	
	mavlink_out_stream = transmit_stream;
	mavlink_in_stream = receive_stream;
	
	init_scheduler(&mavlink_tasks);
	
	add_task(&mavlink_tasks, 100000, RUN_REGULAR, &send_scheduled_parameters, MAVLINK_MSG_ID_PARAM_VALUE);

	centralData = get_central_data();
}

task_return_t mavlink_protocol_update() {
	mavlink_receive_handler();
	if ((mavlink_out_stream->buffer_empty(mavlink_out_stream->data))==true) {
		return run_scheduler_update(&mavlink_tasks, ROUND_ROBIN);
		if (mavlink_out_stream->flush!=NULL) mavlink_out_stream->flush;
	}
	
	control_time_out_waypoint_msg(&(centralData->number_of_waypoints),&centralData->waypoint_receiving,&centralData->waypoint_sending);
	
	
	return 0;
}

task_set* get_mavlink_taskset() {
	return &mavlink_tasks;
}

void suspend_downstream(uint32_t delay) {
	int i;
	for (i=0; i<mavlink_tasks.number_of_tasks; i++) {
		suspend_task(&mavlink_tasks.tasks[i], delay);
	}	
}

uint8_t mavlink_receive(byte_stream_t* stream, Mavlink_Received_t* rec) {
	uint8_t byte;
	//dbg_print(" ");
	while(stream->bytes_available(stream->data) > 0) {
		byte = stream->get(stream->data);
		//dbg_print(".");
		//dbg_print_num(byte, 16);
		//dbg_print("\t");
		if(mavlink_parse_char(MAVLINK_COMM_0, byte, &rec->msg, &rec->status)) {
			//dbg_print("\n");
			return 1;
		}
		//dbg_print_num(rec->status.parse_state, 16);
		//dbg_print("\n");
	}
	return 0;
}

void handle_mavlink_message(Mavlink_Received_t* rec) {
	
	switch(rec->msg.msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: { // 21
			mavlink_param_request_list_t request;
			mavlink_msg_param_request_list_decode(&rec->msg, &request);
			
			dbg_print("msg comp id:");
			dbg_print_num(request.target_component,10);
			dbg_print("\n");
			
			// Check if this message is for this system
			if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid) {
				send_all_parameters();
			}				
		}
		break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ: { //20
			mavlink_param_request_read_t request;
			mavlink_msg_param_request_read_decode(&rec->msg, &request);
			// Check if this message is for this system and subsystem
			if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid
			&& (uint8_t)request.target_component == (uint8_t)mavlink_system.compid) {

				send_parameter(&request);
			}				
		}
		break;
		case MAVLINK_MSG_ID_PARAM_SET: { //23
			suspend_downstream(100000);
			receive_parameter(rec);
		}
		break;
		case MAVLINK_MSG_ID_MISSION_ITEM: { // 39
			suspend_downstream(500000);
			receive_waypoint(rec, centralData->waypoint_list, centralData->number_of_waypoints,&centralData->waypoint_receiving);
		}
		break;
		case MAVLINK_MSG_ID_MISSION_REQUEST : { // 40
			suspend_downstream(500000);
			send_waypoint(rec, centralData->waypoint_list, centralData->number_of_waypoints,&centralData->waypoint_sending);
		}
		break;
		case MAVLINK_MSG_ID_MISSION_SET_CURRENT : { // 41
			set_current_wp(rec, &(centralData->waypoint_list), centralData->number_of_waypoints);
		}
		break;
		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: { // 43
			// this initiates all waypoints being sent to the base-station - therefore, we pause the downstream telemetry to free the channel
			// (at least until we have a radio system with guaranteed bandwidth)
			suspend_downstream(500000);
			send_count(rec, centralData->number_of_waypoints,&centralData->waypoint_receiving,&centralData->waypoint_sending);
		}
		break;
		case MAVLINK_MSG_ID_MISSION_COUNT : { // 44
			// this initiates all waypoints being sent from base-station - therefore, we pause the downstream telemetry to free the channel
			// (at least until we have a radio system with guaranteed bandwidth)
			suspend_downstream(500000);
			receive_count(rec, &(centralData->number_of_waypoints),&centralData->waypoint_receiving,&centralData->waypoint_sending);
		}
		break;
		case MAVLINK_MSG_ID_MISSION_CLEAR_ALL : { // 45
			clear_waypoint_list(rec, &(centralData->number_of_waypoints),&centralData->waypoint_set);
		}
		break;
		case MAVLINK_MSG_ID_MISSION_ACK : { // 47
			receive_ack_msg(rec,&centralData->waypoint_sending);
		}
		break;
		case MAVLINK_MSG_ID_SET_MODE : { // 11
			set_mav_mode(rec, &centralData->mav_mode, &(centralData->mav_state),centralData->simulation_mode);
		}
		break;
		case MAVLINK_MSG_ID_COMMAND_LONG : { // 76
			receive_message_long(rec);
		}
		break;		
		case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: { // 66
			mavlink_request_data_stream_t request;
			mavlink_msg_request_data_stream_decode(&rec->msg, &request);
			// TODO: control target_component == compid!
			if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid
			&& (uint8_t)request.target_component == (uint8_t)mavlink_system.compid)
			{
				dbg_print("stream request:");
				dbg_print_num(request.target_component,10);
				if (request.req_stream_id==255) {
					int i;
					dbg_print("send all\n");
					// send full list of streams
					for (i=0; i<mavlink_tasks.number_of_tasks; i++) {
						run_task_now(&mavlink_tasks.tasks[i]);
					}					
				} else {
					task_entry *task=get_task_by_id(&mavlink_tasks, request.req_stream_id);
					dbg_print(" stream="); dbg_print_num(request.req_stream_id, 10);
					dbg_print(" start_stop=");dbg_print_num(request.start_stop, 10);
					dbg_print(" rate=");dbg_print_num(request.req_message_rate,10);
					dbg_print("\n");
					if (request.start_stop) {
						change_run_mode(task, RUN_REGULAR);
					}else {
						change_run_mode(task, RUN_NEVER);
					}
					if (request.req_message_rate>0) {
						change_task_period(task, SCHEDULER_TIMEBASE/(uint32_t)request.req_message_rate);
					}
				}
		}	
		break;
		//case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: { // 33
			//read_msg_from_neighbors(rec);
			//}			
		//break;
		}		
		/* 
		TODO : add other cases
		*/
	}
}			

void receive_message_long(Mavlink_Received_t* rec)
{
	mavlink_command_long_t packet;
	mavlink_msg_command_long_decode(&rec->msg,&packet);
	// Check if this message is for this system and subsystem
	// dbg_print("target_comp:");
	// dbg_print_num(packet.target_component,10);
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid
	&&(uint8_t)packet.target_component == (uint8_t)0)
	{
		// print packet command and parameters for debug
		dbg_print("parameters:");
		dbg_print_num(packet.param1,10);
		dbg_print_num(packet.param2,10);
		dbg_print_num(packet.param3,10);
		dbg_print_num(packet.param4,10);
		dbg_print_num(packet.param5,10);
		dbg_print_num(packet.param6,10);
		dbg_print_num(packet.param7,10);
		dbg_print(", command id:");
		dbg_print_num(packet.command,10);
		dbg_print(", confirmation:");
		dbg_print_num(packet.confirmation,10);
		dbg_print("\n");
		
		switch(packet.command) {
			case MAV_CMD_NAV_WAYPOINT: {
				/* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|  */
			}
			break;
			case MAV_CMD_NAV_LOITER_UNLIM: {
				/* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			}
			break;
			case MAV_CMD_NAV_LOITER_TURNS: {
				/* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			}
			break;
			case MAV_CMD_NAV_LOITER_TIME: {
				/* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			}
			break;
			case MAV_CMD_NAV_RETURN_TO_LAUNCH: {
				/* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_NAV_LAND: {
				/* Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			}
			break;
			case MAV_CMD_NAV_TAKEOFF: {
				/* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */
			}
			break;
			case MAV_CMD_NAV_ROI: {
				/* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
			}
			break;
			case MAV_CMD_NAV_PATHPLANNING: {
				/* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
			}
			break;
			case MAV_CMD_NAV_LAST: {
				/* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_CONDITION_DELAY: {
				/* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_CONDITION_CHANGE_ALT: {
				/* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
			}
			break;
			case MAV_CMD_CONDITION_DISTANCE: {
				/* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_CONDITION_YAW: {
				/* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_CONDITION_LAST: {
				/* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_DO_SET_MODE: {
				/* Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_DO_JUMP: {
				/* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_DO_CHANGE_SPEED: {
				/* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_DO_SET_HOME: {
				/* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
			}
			break;
			case MAV_CMD_DO_SET_PARAMETER: {
				/* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_DO_SET_RELAY: {
				/* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_DO_REPEAT_RELAY: {
				/* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_DO_SET_SERVO: {
				/* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_DO_REPEAT_SERVO: {
				/* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_DO_CONTROL_VIDEO: {
				/* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_DO_LAST: {
				/* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_PREFLIGHT_CALIBRATION: {
				/* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Accelerometer calibration: 0: no, 1: yes| Empty| Empty|  */
			}
			break;
			case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS: {
				/* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
			}
			break;
			case MAV_CMD_PREFLIGHT_STORAGE: {
				/* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Reserved| Reserved| Empty| Empty| Empty|  */
				
				// Onboard parameters storage
				if (packet.param1 == 0) {
					// read parameters from flash
					dbg_print("Reading from flashc...\n");
					read_parameters_from_flashc();
				}
				else if (packet.param1 == 1) {
					// write parameters to flash
					dbg_print("Writting to flashc\n");
					write_parameters_to_flashc();
				}
				
				// Mission parameters storage
				if (packet.param2 == 0) {
					// read mission from flash
				}
				else if (packet.param2 == 1) {
					// write mission to flash
				}
			}
			break;
			case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: {
				/* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.| Reserved| Reserved| Empty| Empty| Empty|  */
			}
			break;
			case MAV_CMD_OVERRIDE_GOTO: {
				/* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
			}
			break;
			case MAV_CMD_MISSION_START: {
				/* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
			}
			break;
			case MAV_CMD_COMPONENT_ARM_DISARM: {
				/* Arms / Disarms a component |1 to arm, 0 to disarm|  */
			}
			break;
			case MAV_CMD_ENUM_END: {
				/*  | */
			}
			break;
		}
	}
}