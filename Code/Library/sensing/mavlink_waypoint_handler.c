/*
 * mavlink_waypoint_handler.c
 *
 * Created: May 16 2013 17:54:34
 *  Author: Nicolas
 */ 

#include "mavlink_waypoint_handler.h"
#include "print_util.h"
#include "remote_controller.h"
#include "time_keeper.h"
#include "central_data.h"
#include "maths.h"

central_data_t *centralData;

int int_loop_count = 0;

void init_waypoint_handler()
{
	start_timeout = get_millis();
	timeout_max_wp = 10000;
	centralData = get_central_data();
	
	centralData->critical_behavior = CLIMB_TO_SAFE_ALT;
	centralData->critical_init = false;
	centralData->critical_next_state = false;
	
	init_waypoint_list(centralData->waypoint_list, &centralData->number_of_waypoints);
	init_wp();
}

void init_wp()
{
	uint8_t i,j;
	float rel_pos[3];
	
	if (int_loop_count==0)
	{
		dbg_print("Nav init\n");
	}
	int_loop_count=(int_loop_count+1)%1000;
	
	if ((centralData->number_of_waypoints > 0) && (centralData->position_estimator.init_gps_position || centralData->simulation_mode) && centralData->waypoint_receiving == false)
	{
		for (i=0;i<centralData->number_of_waypoints;i++)
		{
			if ((centralData->waypoint_list[i].current == 1)&&(!centralData->waypoint_set))
			{
				centralData->current_wp_count = i;
				centralData->current_waypoint = centralData->waypoint_list[centralData->current_wp_count];
				centralData->waypoint_coordinates = set_waypoint_from_frame(centralData->current_waypoint,centralData->position_estimator.localPosition.origin);
				
				dbg_print("Waypoint Nr");
				dbg_print_num(i,10);
				dbg_print(" set,\n");
			
				centralData->waypoint_set = true;
				//waypoint_reached = false;
				
				for (j=0;j<3;j++)
				{
					rel_pos[j] = centralData->waypoint_coordinates.pos[j]-centralData->position_estimator.localPosition.pos[j];
				}
				centralData->dist2wp_sqr = vector_norm_sqr(rel_pos);
			}
		}
	}
}


void init_waypoint_list(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints)
{
	
	
	// Visit https://code.google.com/p/ardupilot-mega/wiki/MAVLink to have a description of all messages (or common.h)
	waypoint_struct waypoint;
	*number_of_waypoints = 4;
	
	num_waypoint_onboard = *number_of_waypoints;
	
	// Set nav waypoint
	waypoint.autocontinue = 1;
	waypoint.current = 1;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.wp_id = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x =  465185223.6174 / 1.0e7f; // convert to deg
	waypoint.y = 65670560 / 1.0e7f; // convert to deg
	waypoint.z = 20; //m
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 2; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 90; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_list[0] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 1;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.wp_id = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x = 465186816 / 1.0e7f; // convert to deg
	waypoint.y = 65670560 / 1.0e7f; // convert to deg
	waypoint.z = 20; //m
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 4; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 90; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_list[1] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 1;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.wp_id = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x = 465186816 / 1.0e7f; // convert to deg
	waypoint.y = 65659084 / 1.0e7f; // convert to deg
	waypoint.z = 40; //m
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 15; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 90; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_list[2] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 1;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.wp_id = MAV_CMD_NAV_WAYPOINT;

	waypoint.x = 465182186 / 1.0e7f; // convert to deg
	waypoint.y = 65659084 / 1.0e7f; // convert to deg
	waypoint.z = 20; //m

	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 12; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 90; // Desired yaw angle at MISSION (rotary wing)

	waypoint_list[3] = waypoint;
	
	// Set home waypoint
	//waypoint.autocontinue = 1;
	//waypoint.current = 0;
	//waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // 0:Global, 1:local NED, 2:mission, 3:global rel alt, 4: local ENU
	//waypoint.wp_id = MAV_CMD_DO_SET_HOME;
//
	//
	//waypoint.param1 = 0;
	//waypoint.param2 = 20; // altitude
	//waypoint.param3 = 465186806 / 1.0e7f; // lat converted to deg
	//waypoint.param4 = 65659084 / 1.0e7f; // long converted to deg
	
	//waypoint_list[4] = waypoint;
	
	dbg_print("Number of Waypoint onboard:");
	dbg_print_num(num_waypoint_onboard,10);
	dbg_print("\n");
	
}

void send_count(Mavlink_Received_t* rec, uint16_t num_of_waypoint, bool* waypoint_receiving, bool * waypoint_sending)
{
	mavlink_mission_request_list_t packet;
	mavlink_msg_mission_request_list_decode(&rec->msg,&packet);
	
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{	
		mavlink_msg_mission_count_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,num_of_waypoint);
		
		if (num_of_waypoint != 0)
		{
			*waypoint_sending = true;
			*waypoint_receiving = false;
			start_timeout = get_millis();
		}
		
		sending_wp_num = 0;
		dbg_print("Will send ");
		dbg_print_num(num_of_waypoint,10);
		dbg_print(" waypoints\n");
	}
}

void send_waypoint(Mavlink_Received_t* rec, waypoint_struct waypoint[], uint16_t num_of_waypoint, bool* waypoint_sending)
{
	if (*waypoint_sending)
	{
		mavlink_mission_request_t packet;
		mavlink_msg_mission_request_decode(&rec->msg,&packet);
		
		dbg_print("Asking for waypoint number ");
		dbg_print_num(packet.seq,10);
		dbg_print("\n");
		
		// Check if this message is for this system and subsystem
		if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
		&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
		{
			sending_wp_num = packet.seq;
			if (sending_wp_num < num_of_waypoint)
			{
				// mavlink_msg_mission_item_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
				mavlink_msg_mission_item_send(MAVLINK_COMM_0, rec->msg.sysid, rec->msg.compid, packet.seq,
				waypoint[sending_wp_num].frame,   waypoint[sending_wp_num].wp_id,
				waypoint[sending_wp_num].current, waypoint[sending_wp_num].autocontinue,
				waypoint[sending_wp_num].param1,  waypoint[sending_wp_num].param2,       waypoint[sending_wp_num].param3,    waypoint[sending_wp_num].param4,
				waypoint[sending_wp_num].x,       waypoint[sending_wp_num].y,            waypoint[sending_wp_num].z);
				
				dbg_print("Sending waypoint ");
				dbg_print_num(sending_wp_num, 10);
				dbg_print("\n");
				
				start_timeout = get_millis();
				
				//sending_wp_num += 1;
			}			
		}
	}	
}

void receive_ack_msg(Mavlink_Received_t* rec, bool* waypoint_sending)
{
	mavlink_mission_ack_t packet;
	mavlink_msg_mission_ack_decode(&rec->msg, &packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		*waypoint_sending = false;
		sending_wp_num = 0;
		dbg_print("Acknowledgment received, end of waypoint sending.\n");
	}
}

void receive_count(Mavlink_Received_t* rec, uint16_t* number_of_waypoints, bool* waypoint_receiving, bool* waypoint_sending)
{
	mavlink_mission_count_t packet;
	mavlink_msg_mission_count_decode(&rec->msg, &packet);
	// Check if this message is for this system and subsystem
	//dbg_print("check msg");
	//dbg_print_num(packet.target_system,10);
	//dbg_print_num(mavlink_mission_planner.sysid,10);
	//dbg_print_num(packet.target_component,10);
	//dbg_print_num(mavlink_mission_planner.compid,10);
	//dbg_print("\n");
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_mission_planner.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		if (*waypoint_receiving == false)
		{
			// comment these lines if you want to add new waypoints to the list instead of overwriting them
			num_waypoint_onboard = 0;
			*number_of_waypoints =0;
			//---//
			
			if ((packet.count + *number_of_waypoints) > MAX_WAYPOINTS)
			{
				packet.count = MAX_WAYPOINTS - *number_of_waypoints;
			}
			*number_of_waypoints =  packet.count+ *number_of_waypoints;
			dbg_print("Receiving ");
			dbg_print_num(packet.count,10);
			dbg_print(" new waypoints. ");
			dbg_print("New total number of waypoints:");
			dbg_print_num(*number_of_waypoints,10);
			dbg_print("\n");
			
			*waypoint_receiving   = true;
			*waypoint_sending     = false;
			waypoint_request_number = 0;
			
			
			start_timeout = get_millis();
		}
		
		mavlink_msg_mission_request_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,waypoint_request_number);
		
		dbg_print("Asking for waypoint ");
		dbg_print_num(waypoint_request_number,10);
		dbg_print("\n");	
	}
	
}

void receive_waypoint(Mavlink_Received_t* rec,  waypoint_struct waypoint_list[], uint16_t number_of_waypoints, bool* waypoint_receiving)
{
	mavlink_mission_item_t packet;
	mavlink_msg_mission_item_decode(&rec->msg,&packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		start_timeout = get_millis();
		
		waypoint_struct new_waypoint;
		
		new_waypoint.wp_id = packet.command;
		
		new_waypoint.x = packet.x; // longitude
		new_waypoint.y = packet.y; // latitude 
		new_waypoint.z = packet.z; // altitude
		
		new_waypoint.autocontinue = packet.autocontinue;
		new_waypoint.frame = packet.frame;
		
		new_waypoint.current = packet.current;
		
		new_waypoint.param1 = packet.param1;
		new_waypoint.param2 = packet.param2;
		new_waypoint.param3 = packet.param3;
		new_waypoint.param4 = packet.param4;
		
		dbg_print("New waypoint received ");
		//dbg_print("(");
 		//dbg_print_num(new_waypoint.x,10);
 		//dbg_print(", ");
 		//dbg_print_num(new_waypoint.y,10);
 		//dbg_print(", ");
 		//dbg_print_num(new_waypoint.z,10);
 		//dbg_print(") Autocontinue:");
 		//dbg_print_num(new_waypoint.autocontinue,10);
 		//dbg_print(" Frame:");
 		//dbg_print_num(new_waypoint.frame,10);
 		//dbg_print(" Current :");
 		//dbg_print_num(packet.current,10);
 		//dbg_print(" Seq :");
 		//dbg_print_num(packet.seq,10);
		//dbg_print(" command id :");
		//dbg_print_num(packet.command,10);
		dbg_print(" requested num :");
		dbg_print_num(waypoint_request_number,10);
		dbg_print(" receiving num :");
		dbg_print_num(packet.seq,10);
		//dbg_print(" is it receiving :");
		//dbg_print_num(waypoint_receiving,10); // boolean value
		dbg_print("\n");
		
		//switch(packet.command)
		//{
			//case MAV_CMD_NAV_LOITER_TURNS:
			//case MAV_CMD_DO_SET_HOME:
			//new_waypoint.param1 = packet.param1;
			//break;
//
			//case MAV_CMD_NAV_ROI:
			//new_waypoint.param1 = packet.param1;                                    // MAV_ROI (aka roi mode) is held in wp's parameter but we actually do nothing with it because we only support pointing at a specific location provided by x,y and z parameters
			//break;
//
			//case MAV_CMD_CONDITION_YAW:
			//new_waypoint.param1 = packet.param3;
			//new_waypoint.alt = packet.param1;
			//new_waypoint.lat = packet.param2;
			//new_waypoint.lon = packet.param4;
			//break;
//
			//case MAV_CMD_NAV_TAKEOFF:
			//new_waypoint.param1 = 0;
			//break;
//
			//case MAV_CMD_CONDITION_CHANGE_ALT:
			//new_waypoint.param1 = packet.param1 * 100;
			//break;
//
			//case MAV_CMD_NAV_LOITER_TIME:
			//new_waypoint.param1 = packet.param1;                                    // APM loiter time is in ten second increments
			//break;
//
			//case MAV_CMD_CONDITION_DELAY:
			//case MAV_CMD_CONDITION_DISTANCE:
			//new_waypoint.lat = packet.param1;
			//break;
//
			//case MAV_CMD_DO_JUMP:
			//new_waypoint.lat = packet.param2;
			//new_waypoint.param1  = packet.param1;
			//break;
//
			//case MAV_CMD_DO_REPEAT_SERVO:
			//new_waypoint.lon = packet.param4;
			//case MAV_CMD_DO_REPEAT_RELAY:
			//case MAV_CMD_DO_CHANGE_SPEED:
			//new_waypoint.lat = packet.param3;
			//new_waypoint.alt = packet.param2;
			//new_waypoint.param1 = packet.param1;
			//break;
//
			//case MAV_CMD_NAV_WAYPOINT:
			//new_waypoint.param1 = packet.param1;
			//break;
//
			//case MAV_CMD_DO_SET_PARAMETER:
			//case MAV_CMD_DO_SET_RELAY:
			//case MAV_CMD_DO_SET_SERVO:
			//new_waypoint.alt = packet.param2;
			//new_waypoint.param1 = packet.param1;
			//break;
		//}
		
		if(packet.current == 2) {                                               //current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
			// switch to guided mode
			//set_mode(GUIDED);

			// set wp_nav's destination
			//wp_nav.set_destination(pv_location_to_vector(tell_command));

			// verify we received the command
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, rec->msg.sysid,rec->msg.compid, MAV_CMD_ACK_OK);

		} else if(packet.current == 3){                                    //current = 3 is a flag to tell us this is a alt change only

			// add home alt if needed
			//if (new_waypoint.options & MASK_OPTIONS_RELATIVE_ALT) 
			//{
			//	new_waypoint.alt += home.alt;
			//}

			// To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
			// similar to how do_change_alt works
			//wp_nav.set_desired_alt(new_waypoint.alt);

			// verify we received the command
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, rec->msg.sysid,rec->msg.compid, MAV_CMD_ACK_OK);

		} else {
			// Check if receiving waypoints
			if (*waypoint_receiving){

				// check if this is the requested waypoint
				if (packet.seq == waypoint_request_number)
				{
					dbg_print("Receiving good waypoint, number ");
					dbg_print_num(waypoint_request_number,10);
					dbg_print(" of ");
					dbg_print_num(number_of_waypoints-num_waypoint_onboard,10);
					dbg_print("\n");
					
				    waypoint_list[num_waypoint_onboard + waypoint_request_number] = new_waypoint;
					waypoint_request_number++;
					
					if ((num_waypoint_onboard + waypoint_request_number) == number_of_waypoints) 
					{
						
						uint8_t type = MAV_CMD_ACK_OK;                         // ok (0), error(1) ???
						//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,mavlink_mission_planner.compid,type);
						mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,type);
						//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,0,type);
						//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,mavlink_system.compid,type);
						
						//mavlink_msg_mission_ack_send(MAVLINK_COMM_0, packet.target_system, packet.target_component,type);

						dbg_print("flight plan received!\n");
						*waypoint_receiving = false;
						num_waypoint_onboard = number_of_waypoints;
						centralData->waypoint_set = false;
						init_wp();
					}else{
						mavlink_msg_mission_request_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,waypoint_request_number);
						
						dbg_print("Asking for waypoint ");
						dbg_print_num(waypoint_request_number,10);
						dbg_print("\n");
					}
				}
			}else{
				uint8_t type = MAV_CMD_ACK_OK; //MAV_CMD_ACK_ERR_FAIL;                         // ok (0), error(1)
				dbg_print("Ack not received!");
				mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,type);
			}				
		}		
	}			
}		

void set_current_wp(Mavlink_Received_t* rec,  waypoint_struct waypoint_list[], uint16_t num_of_waypoint)
{
	mavlink_mission_set_current_t packet;
	mavlink_msg_mission_set_current_decode(&rec->msg,&packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		if (packet.seq < num_of_waypoint)
		{
			//dbg_print("setting current wp\n");
			int i;
			for (i=0;i<num_of_waypoint;i++)
			{
				waypoint_list[i].current = 0;
			}
			
			waypoint_list[packet.seq].current = 1;
			mavlink_msg_mission_current_send(MAVLINK_COMM_0,waypoint_list[packet.seq].current);
			
			dbg_print("Set current waypoint to number");
			dbg_print_num(packet.seq,10);
			dbg_print("\n");
			
			centralData->waypoint_set = false;
			init_wp();
		}else{
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_ACCESS_DENIED);
		}
	}
}

void clear_waypoint_list(Mavlink_Received_t* rec,  uint16_t* number_of_waypoints, bool* waypoint_set)
{
	mavlink_mission_clear_all_t packet;
	mavlink_msg_mission_clear_all_decode(&rec->msg,&packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid
	&& (uint8_t)packet.target_component == (uint8_t)mavlink_mission_planner.compid)
	{
		*number_of_waypoints = 0;
		num_waypoint_onboard = 0;
		*waypoint_set = 0;
		mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_OK);
		dbg_print("Clear Waypoint list");
	}		
}

void set_mav_mode(Mavlink_Received_t* rec, uint8_t* board_mav_mode, uint8_t* board_mav_state, uint8_t sim_mode)
{
	mavlink_set_mode_t packet;
	mavlink_msg_set_mode_decode(&rec->msg,&packet);
	// Check if this message is for this system and subsystem
	if ((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid)
	{
		dbg_print("base_mode:");
		dbg_print_num(packet.base_mode,10);
		dbg_print(", custom mode:");
		dbg_print_num(packet.custom_mode,10);
		dbg_print("\n");

		if (sim_mode == 0)
		{
			switch(packet.base_mode)
			{
				case MAV_MODE_STABILIZE_DISARMED:
				case MAV_MODE_GUIDED_DISARMED:
				case MAV_MODE_AUTO_DISARMED:
					*board_mav_state = MAV_STATE_STANDBY;
					*board_mav_mode = MAV_MODE_MANUAL_DISARMED;
				break;
				case MAV_MODE_MANUAL_ARMED:
					if (get_thrust_from_remote()<-0.95)
					{
						*board_mav_state = MAV_STATE_ACTIVE;
						*board_mav_mode = MAV_MODE_MANUAL_ARMED;
					}
				break;
			}
		}else{
			switch(packet.base_mode)
			{
				case MAV_MODE_STABILIZE_DISARMED:
				case MAV_MODE_GUIDED_DISARMED:
				case MAV_MODE_AUTO_DISARMED:
					*board_mav_state = MAV_STATE_STANDBY;
					*board_mav_mode = MAV_MODE_MANUAL_DISARMED;
				break;
				case MAV_MODE_MANUAL_ARMED:
					*board_mav_state = MAV_STATE_ACTIVE;
					*board_mav_mode = MAV_MODE_MANUAL_ARMED;
				break;
				case MAV_MODE_STABILIZE_ARMED:
					*board_mav_state = MAV_STATE_ACTIVE;
					*board_mav_mode = MAV_MODE_STABILIZE_ARMED;
				break;
				case MAV_MODE_GUIDED_ARMED:
					*board_mav_state = MAV_STATE_ACTIVE;
					*board_mav_mode = MAV_MODE_GUIDED_ARMED;
				break;
				case MAV_MODE_AUTO_ARMED:
					*board_mav_state = MAV_STATE_ACTIVE;
					*board_mav_mode = MAV_MODE_AUTO_ARMED;
				break;
			}
		}
		
	}
}

void control_time_out_waypoint_msg(uint16_t* num_of_waypoint, bool* waypoint_receiving, bool* waypoint_sending)
{
	if (*waypoint_sending || *waypoint_receiving)
	{
		uint32_t tnow = get_millis();
		
		if ((tnow - start_timeout) > timeout_max_wp)
		{
			start_timeout = tnow;
			if (*waypoint_sending)
			{
				*waypoint_sending = false;
				dbg_print("Sending waypoint timeout");
			}
			if (*waypoint_receiving)
			{
				*waypoint_receiving = false;
				
				dbg_print("Receiving waypoint timeout");
				*num_of_waypoint = 0;
				num_waypoint_onboard = 0;
				
			}
		}
	}
}

local_coordinates_t set_waypoint_from_frame(waypoint_struct current_wp, global_position_t origin)
{
	uint8_t i;
	
	global_position_t waypoint_global;
	local_coordinates_t waypoint_coor;
	
	float rel_pos[3];
	
	for (i=0;i<3;i++)
	{
		waypoint_coor.pos[i] = 0.0;
	}

	switch(current_wp.frame)
	{
		case MAV_FRAME_GLOBAL:
			waypoint_global.latitude = current_wp.x;
			waypoint_global.longitude = current_wp.y;
			waypoint_global.altitude = current_wp.z;
			waypoint_coor = global_to_local_position(waypoint_global,origin);
			
			dbg_print("wp_global: lat (x1e7):");
			dbg_print_num(waypoint_global.latitude*10000000,10);
			dbg_print(" long (x1e7):");
			dbg_print_num(waypoint_global.longitude*10000000,10);
			dbg_print(" alt (x1000):");
			dbg_print_num(waypoint_global.altitude*1000,10);
			dbg_print(" wp_coor: x (x100):");
			dbg_print_num(waypoint_coor.pos[X]*100,10);
			dbg_print(", y (x100):");
			dbg_print_num(waypoint_coor.pos[Y]*100,10);
			dbg_print(", z (x100):");
			dbg_print_num(waypoint_coor.pos[Z]*100,10);
			dbg_print(" localOrigin lat (x1e7):");
			dbg_print_num(origin.latitude*10000000,10);
			dbg_print(" long (x1e7):");
			dbg_print_num(origin.longitude*10000000,10);
			dbg_print(" alt (x1000):");
			dbg_print_num(origin.altitude*1000,10);
			dbg_print("\n");
		
		break;
		case MAV_FRAME_LOCAL_NED:
			waypoint_coor.pos[X] = current_wp.x;
			waypoint_coor.pos[Y] = current_wp.y;
			waypoint_coor.pos[Z] = current_wp.z;
			waypoint_coor.heading= deg_to_rad(current_wp.param4);
			waypoint_coor.origin = local_to_global_position(waypoint_coor);
		break;
		case MAV_FRAME_MISSION:
			//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_RESULT_UNSUPPORTED);
		break;
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
			waypoint_global.latitude = current_wp.x;
			waypoint_global.longitude = current_wp.y;
			waypoint_global.altitude = current_wp.z;
		
			global_position_t origin_relative_alt = origin;
			origin_relative_alt.altitude = 0.0;
			waypoint_coor = global_to_local_position(waypoint_global,origin_relative_alt);
		
			dbg_print("LocalOrigin: lat (x1e7):");
			dbg_print_num(origin_relative_alt.latitude * 10000000,10);
			dbg_print(" long (x1e7):");
			dbg_print_num(origin_relative_alt.longitude * 10000000,10);
			dbg_print(" global alt (x1000):");
			dbg_print_num(origin.altitude*1000,10);
			dbg_print(" wp_coor: x (x100):");
			dbg_print_num(waypoint_coor.pos[X]*100,10);
			dbg_print(", y (x100):");
			dbg_print_num(waypoint_coor.pos[Y]*100,10);
			dbg_print(", z (x100):");
			dbg_print_num(waypoint_coor.pos[Z]*100,10);
			dbg_print("\n");
		
		break;
		case MAV_FRAME_LOCAL_ENU:
			//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_RESULT_UNSUPPORTED);
		break;
	}
	
	return waypoint_coor;
}

void wp_hold_init()
{
	if (centralData->waypoint_hold_init == 0)
	{
		dbg_print("Position hold at: ");
		dbg_print_num(centralData->position_estimator.localPosition.pos[X],10);
		dbg_print_num(centralData->position_estimator.localPosition.pos[Y],10);
		dbg_print_num(centralData->position_estimator.localPosition.pos[Z],10);
		dbg_print_num((int)(centralData->position_estimator.localPosition.heading*180.0/3.14),10);
		dbg_print(")\n");
		
		centralData->waypoint_hold_init = true;
		centralData->waypoint_hold_coordinates = centralData->position_estimator.localPosition;
		
		Aero_Attitude_t aero_attitude;
		aero_attitude=Quat_to_Aero(centralData->imu1.attitude.qe);
		centralData->waypoint_hold_coordinates.heading = aero_attitude.rpy[2];
	}
}

void waypoint_hold_position_handler()
{
	if (!centralData->waypoint_set)
	{
		init_wp();
	}
	wp_hold_init();
}

void waypoint_navigation_handler()
{

	if (centralData->waypoint_set)
	{
		uint8_t i;
		float rel_pos[3];
		for (i=0;i<3;i++)
		{
			rel_pos[i] = centralData->waypoint_coordinates.pos[i]-centralData->position_estimator.localPosition.pos[i];
		}
		centralData->dist2wp_sqr = vector_norm_sqr(rel_pos);
		
		if (centralData->dist2wp_sqr < (centralData->current_waypoint.param2*centralData->current_waypoint.param2))
		{
			dbg_print("Waypoint Nr");
			dbg_print_num(centralData->current_wp_count,10);
			dbg_print(" reached, distance:");
			dbg_print_num(sqrt(centralData->dist2wp_sqr),10);
			dbg_print(" less than :");
			dbg_print_num(centralData->current_waypoint.param2,10);
			dbg_print(".\n");
			mavlink_msg_mission_item_reached_send(MAVLINK_COMM_0,centralData->current_wp_count);
			
			centralData->waypoint_list[centralData->current_wp_count].current = 0;
			if (centralData->current_waypoint.autocontinue == 1)
			{
				dbg_print("Autocontinue towards waypoint Nr");
				
				if (centralData->current_wp_count == (centralData->number_of_waypoints-1))
				{
					centralData->current_wp_count = 0;
					}else{
					centralData->current_wp_count++;
				}
				dbg_print_num(centralData->current_wp_count,10);
				dbg_print("\n");
				centralData->waypoint_list[centralData->current_wp_count].current = 1;
				centralData->current_waypoint = centralData->waypoint_list[centralData->current_wp_count];
				centralData->waypoint_coordinates = set_waypoint_from_frame(centralData->current_waypoint,centralData->position_estimator.localPosition.origin);
				
				mavlink_msg_mission_current_send(MAVLINK_COMM_0,centralData->current_wp_count);
				
				}else{
				centralData->waypoint_set = false;
				dbg_print("Stop\n");
				
				wp_hold_init();
			}
		}
	}else{
		init_wp();
		wp_hold_init();
	}
}

void waypoint_critical_handler()
{
	if (!(centralData->critical_init))
	{
		centralData->critical_init = true;
		dbg_print("Critical State! Climbing to safe altitude.\n");
		centralData->critical_behavior = CLIMB_TO_SAFE_ALT;
	}
	
	if (!(centralData->critical_next_state))
	{
		centralData->critical_next_state = true;
		
		Aero_Attitude_t aero_attitude;
		aero_attitude=Quat_to_Aero(centralData->imu1.attitude.qe);
		centralData->waypoint_critical_coordinates.heading = aero_attitude.rpy[2];
		
		switch (centralData->critical_behavior)
		{
			case CLIMB_TO_SAFE_ALT:
			centralData->waypoint_critical_coordinates.pos[X] = centralData->position_estimator.localPosition.pos[X];
			centralData->waypoint_critical_coordinates.pos[Y] = centralData->position_estimator.localPosition.pos[Y];
			centralData->waypoint_critical_coordinates.pos[Z] = -30.0;
			
			break;
			case FLY_TO_HOME_WP:
			centralData->waypoint_critical_coordinates.pos[X] = 0.0;
			centralData->waypoint_critical_coordinates.pos[Y] = 0.0;
			centralData->waypoint_critical_coordinates.pos[Z] = -30.0;
			break;
			case CRITICAL_LAND:
			centralData->waypoint_critical_coordinates.pos[X] = 0.0;
			centralData->waypoint_critical_coordinates.pos[Y] = 0.0;
			centralData->waypoint_critical_coordinates.pos[Z] = 0.0;
			break;
		}
		float rel_pos[3];
		uint8_t i;
		for (i=0;i<3;i++)
		{
			rel_pos[i] = centralData->waypoint_critical_coordinates.pos[i] - centralData->position_estimator.localPosition.pos[i];
		}
		centralData->dist2wp_sqr = vector_norm_sqr(rel_pos);
	}
	
	if (centralData->dist2wp_sqr < 3.0)
	{
		centralData->critical_next_state = false;
		switch (centralData->critical_behavior)
		{
			case CLIMB_TO_SAFE_ALT:
			dbg_print("Critical State! Flying to home waypoint.\n");
			centralData->critical_behavior = FLY_TO_HOME_WP;
			break;
			case FLY_TO_HOME_WP:
			dbg_print("Critical State! Performing critical landing.\n");
			centralData->critical_behavior = CRITICAL_LAND;
			break;
			case CRITICAL_LAND:
			dbg_print("Critical State! Landed, switching off motors, Emergency mode.\n");
			centralData->critical_landing = true;
			break;
		}
	}
}