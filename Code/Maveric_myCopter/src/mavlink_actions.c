/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 * 
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file mavlink_action.c
 *
 * Definition of the tasks executed on the autopilot
 */ 


#include "mavlink_actions.h"
#include "central_data.h"

#include "onboard_parameters.h"
#include "scheduler.h"
#include "mavlink_waypoint_handler.h"
#include "neighbor_selection.h"


central_data_t *centralData;


void mavlink_actions_add_onboard_parameters(void) {
	Stabiliser_t* rate_stabiliser = &centralData->stabiliser_stack.rate_stabiliser;
	Stabiliser_t* attitude_stabiliser = &centralData->stabiliser_stack.attitude_stabiliser;
	Stabiliser_t* velocity_stabiliser= &centralData->stabiliser_stack.velocity_stabiliser;

	// Comp and sys ID
	// onboard_parameters_add_parameter_uint8(&(mavlink_system.sysid),"ID_System");
	// onboard_parameters_add_parameter_uint8(&(mavlink_mission_planner.sysid),"ID_Planner");
	
	// Simulation mode
	onboard_parameters_add_parameter_int32((int32_t*)&centralData->simulation_mode, "Sim_mode");
	
	// Roll rate PID
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].p_gain, "RollRPid_P_G");
	//onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.maths_clip, "RollRPid_I_CLip");
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.postgain, "RollRPid_I_PstG");
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.pregain, "RollRPid_I_PreG");
	//onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.maths_clip, "RollRPid_D_Clip");
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.gain, "RollRPid_D_Gain");
	//onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.LPF, "RollRPid_D_LPF");
	
	// Roll attitude PID
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].p_gain, "RollAPid_P_G");
	//onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.maths_clip, "RollAPid_I_CLip");
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.postgain, "RollAPid_I_PstG");
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.pregain, "RollAPid_I_PreG");
	//onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.maths_clip, "RollAPid_D_Clip");
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.gain, "RollAPid_D_Gain");
	//onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.LPF, "RollAPid_D_LPF");

	// Pitch rate PID
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].p_gain, "PitchRPid_P_G");
	//onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.maths_clip, "PitchRPid_I_CLip");
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.postgain, "PitchRPid_I_PstG");
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.pregain, "PitchRPid_I_PreG");
	//onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.maths_clip, "PitchRPid_D_Clip");
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.gain, "PitchRPid_D_Gain");
	//onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.LPF, "PitchRPid_D_LPF");
	
	// Pitch attitude PID
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].p_gain, "PitchAPid_P_G");
	//onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.maths_clip, "PitchAPid_I_CLip");
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.postgain, "PitchAPid_I_PstG");
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.pregain, "PitchAPid_I_PreG");
	//onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.maths_clip, "PitchAPid_D_Clip");
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.gain, "PitchAPid_D_Gain");
	//onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.LPF, "PitchAPid_D_LPF");

	// Yaw rate PID
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[YAW].p_gain, "YawRPid_P_G");
	//onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[YAW].clip_max, "YawRPid_P_CLmx");
	//onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[YAW].clip_min, "YawRPid_P_CLmn");
	//onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.maths_clip, "YawRPid_I_CLip");
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.postgain, "YawRPid_I_PstG");
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.pregain, "YawRPid_I_PreG");
	//onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.maths_clip, "YawRPid_D_Clip");
	onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.gain, "YawRPid_D_Gain");
	//onboard_parameters_add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.LPF, "YawRPid_D_LPF");
	
	// Yaw attitude PID
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].p_gain, "YawAPid_P_G");
	//onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].clip_max, "YawAPid_P_CLmx");
	//onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].clip_min, "YawAPid_P_CLmn");
	//onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.maths_clip, "YawAPid_I_CLip");
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.postgain, "YawAPid_I_PstG");
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.pregain, "YawAPid_I_PreG");
	//onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.maths_clip, "YawAPid_D_Clip");
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.gain, "YawAPid_D_Gain");
	//onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.LPF, "YawAPid_D_LPF");


	// Roll velocity PID
	onboard_parameters_add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].p_gain, "RollVPid_P_G");
	onboard_parameters_add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].integrator.postgain, "RollVPid_I_PstG");
	onboard_parameters_add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].integrator.pregain, "RollVPid_I_PreG");
	onboard_parameters_add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].differentiator.gain, "RollVPid_D_Gain");

	// Pitch velocity PID
	onboard_parameters_add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].p_gain, "PitchVPid_P_G");
	onboard_parameters_add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].integrator.postgain, "PitchVPid_I_PstG");
	onboard_parameters_add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].integrator.pregain, "PitchVPid_I_PreG");
	onboard_parameters_add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].differentiator.gain, "PitchVPid_D_Gain");

	// Thrust velocity PID
	onboard_parameters_add_parameter_float(&velocity_stabiliser->thrust_controller.p_gain, "ThrVPid_P_G");
	onboard_parameters_add_parameter_float(&velocity_stabiliser->thrust_controller.integrator.postgain, "ThrVPid_I_PstG");
	onboard_parameters_add_parameter_float(&velocity_stabiliser->thrust_controller.integrator.pregain, "ThrVPid_I_PreG");
	onboard_parameters_add_parameter_float(&velocity_stabiliser->thrust_controller.differentiator.gain, "ThrVPid_D_Gain");
	onboard_parameters_add_parameter_float(&velocity_stabiliser->thrust_controller.differentiator.LPF, "ThrVPid_D_LPF");
	onboard_parameters_add_parameter_float(&velocity_stabiliser->thrust_controller.soft_zone_width, "ThrVPid_soft");

	// qfilter
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.kp, "QF_kp_acc");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.kp_mag, "QF_kp_mag");
	onboard_parameters_add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.gain, "YawAPid_D_Gain");
	
	// Biaises
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.be[GYRO_OFFSET + X],"Bias_Gyro_X");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.be[GYRO_OFFSET + Y],"Bias_Gyro_Y");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.be[GYRO_OFFSET + Z],"Bias_Gyro_Z");
	
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.be[ACC_OFFSET + X],"Bias_Acc_X");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.be[ACC_OFFSET + Y],"Bias_Acc_Y");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.be[ACC_OFFSET + Z],"Bias_Acc_Z");
	
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.be[MAG_OFFSET + X],"Bias_Mag_X");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.be[MAG_OFFSET + Y],"Bias_Mag_Y");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.be[MAG_OFFSET + Z],"Bias_Mag_Z");
	
	// Scale factor
	//onboard_parameters_add_parameter_float(&centralData->imu1.raw_scale[GYRO_OFFSET + X],"Scale_Gyro_X");
	//onboard_parameters_add_parameter_float(&centralData->imu1.raw_scale[GYRO_OFFSET + Y],"Scale_Gyro_Y");
	//onboard_parameters_add_parameter_float(&centralData->imu1.raw_scale[GYRO_OFFSET + Z],"Scale_Gyro_Z");
	//
	//onboard_parameters_add_parameter_float(&centralData->imu1.raw_scale[ACC_OFFSET + X],"Scale_Acc_X");
	//onboard_parameters_add_parameter_float(&centralData->imu1.raw_scale[ACC_OFFSET + Y],"Scale_Acc_Y");
	//onboard_parameters_add_parameter_float(&centralData->imu1.raw_scale[ACC_OFFSET + Z],"Scale_Acc_Z");
	//
	//onboard_parameters_add_parameter_float(&centralData->imu1.raw_scale[MAG_OFFSET + X],"Scale_Mag_X");
	//onboard_parameters_add_parameter_float(&centralData->imu1.raw_scale[MAG_OFFSET + Y],"Scale_Mag_Y");
	//onboard_parameters_add_parameter_float(&centralData->imu1.raw_scale[MAG_OFFSET + Z],"Scale_Mag_Z");
	
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.sf[GYRO_OFFSET + X],"Scale_Gyro_X");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.sf[GYRO_OFFSET + Y],"Scale_Gyro_Y");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.sf[GYRO_OFFSET + Z],"Scale_Gyro_Z");
	
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.sf[ACC_OFFSET + X],"Scale_Acc_X");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.sf[ACC_OFFSET + Y],"Scale_Acc_Y");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.sf[ACC_OFFSET + Z],"Scale_Acc_Z");
	
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.sf[MAG_OFFSET + X],"Scale_Mag_X");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.sf[MAG_OFFSET + Y],"Scale_Mag_Y");
	onboard_parameters_add_parameter_float(&centralData->imu1.attitude.sf[MAG_OFFSET + Z],"Scale_Mag_Z");

	//onboard_parameters_add_parameter_float(&centralData->position_estimator.kp_alt,"Pos_kp_alt");
	//onboard_parameters_add_parameter_float(&centralData->position_estimator.kp_vel_baro,"Pos_kp_velb");
	//onboard_parameters_add_parameter_float(&centralData->position_estimator.kp_pos[0],"Pos_kp_pos0");
	//onboard_parameters_add_parameter_float(&centralData->position_estimator.kp_pos[1],"Pos_kp_pos1");
	//onboard_parameters_add_parameter_float(&centralData->position_estimator.kp_pos[2],"Pos_kp_pos2");
	
	onboard_parameters_add_parameter_float(&centralData->dist2vel_gain,"vel_dist2Vel");
	onboard_parameters_add_parameter_float(&centralData->cruise_speed,"vel_cruiseSpeed");
	onboard_parameters_add_parameter_float(&centralData->max_climb_rate,"vel_climbRate");
	onboard_parameters_add_parameter_float(&centralData->softZoneSize,"vel_softZone");
}

void mavlink_actions_handle_specific_messages (Mavlink_Received_t* rec) 
{
	if (rec->msg.sysid == MAVLINK_BASE_STATION_ID) 
	{	
		/*	Use this block for message debugging		
		print_util_dbg_print("\n Received message with ID");
		print_util_dbg_print_num(rec->msg.msgid, 10);
		print_util_dbg_print(" from system");
		print_util_dbg_print_num(rec->msg.sysid, 10);
		print_util_dbg_print(" for component");
		print_util_dbg_print_num(rec->msg.compid,10);
		print_util_dbg_print( "\n");
		*/

		switch(rec->msg.msgid) 
		{
			case MAVLINK_MSG_ID_MISSION_ITEM:	// 39 
				mavlink_stream_suspend_downstream(500000);
				waypoint_handler_receive_waypoint(	rec, 
									centralData->waypoint_list, 
									centralData->number_of_waypoints,
									&centralData->waypoint_receiving	);
				break;
	
			case MAVLINK_MSG_ID_MISSION_REQUEST : // 40
				mavlink_stream_suspend_downstream(500000);
				waypoint_handler_send_waypoint(	rec, 
								centralData->waypoint_list, 
								centralData->number_of_waypoints,
								&centralData->waypoint_sending	);
				break;

			case MAVLINK_MSG_ID_MISSION_SET_CURRENT :  // 41
				waypoint_handler_set_current_waypoint(	rec, 
										centralData->waypoint_list, 
										centralData->number_of_waypoints	);
				break;

			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:  // 43
				// this initiates all waypoints being sent to the base-station - therefore, we pause the downstream telemetry to free the channel
				// (at least until we have a radio system with guaranteed bandwidth)
				mavlink_stream_suspend_downstream(500000);
				waypoint_handler_send_count(	rec, 
							centralData->number_of_waypoints,
							&centralData->waypoint_receiving,
							&centralData->waypoint_sending	);
				break;

			case MAVLINK_MSG_ID_MISSION_COUNT :  // 44
				// this initiates all waypoints being sent from base-station - therefore, we pause the downstream telemetry to free the channel
				// (at least until we have a radio system with guaranteed bandwidth)
				mavlink_stream_suspend_downstream(500000);
				waypoint_handler_receive_count(	rec, 
								&(centralData->number_of_waypoints),
								&centralData->waypoint_receiving,
								&centralData->waypoint_sending	);
				break;

			case MAVLINK_MSG_ID_MISSION_CLEAR_ALL :  // 45
				waypoint_handler_clear_waypoint_list(	rec, 
										&(centralData->number_of_waypoints),
										&centralData->waypoint_set	);
				break;

			case MAVLINK_MSG_ID_MISSION_ACK :  // 47
				waypoint_handler_receive_ack_msg(	rec,
									&centralData->waypoint_sending	);
				break;

			case MAVLINK_MSG_ID_SET_MODE :  // 11
				waypoint_handler_set_mav_mode(	rec, 
								&centralData->mav_mode, 
								&(centralData->mav_state),
								centralData->simulation_mode	);
				break;

			case MAVLINK_MSG_ID_COMMAND_LONG :  // 76
				mavlink_actions_receive_message_long(rec);
				break;

			case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:  // 48
				waypoint_handler_set_home(rec);
				break;
		}
	} 
	else if (rec->msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
	{
		/* 
		 * Use this block for message debugging
		 * 
		 * print_util_dbg_print("\n Received message with ID");
		 * print_util_dbg_print_num(rec->msg.msgid, 10);
		 * print_util_dbg_print(" from system");
		 * print_util_dbg_print_num(rec->msg.sysid, 10);
		 * print_util_dbg_print(" for component");
		 * print_util_dbg_print_num(rec->msg.compid,10);
		 * print_util_dbg_print( "\n");
		*/

		neighbors_selection_read_message_from_neighbors(rec);
	}
}


void mavlink_actions_receive_message_long(Mavlink_Received_t* rec)
{
	mavlink_command_long_t packet;
	
	mavlink_msg_command_long_decode(&rec->msg, &packet);

	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid) && ((uint8_t)packet.target_component == (uint8_t)0))		// TODO check this 0
	{
		// print packet command and parameters for debug
		print_util_dbg_print("parameters:");
		print_util_dbg_print_num(packet.param1,10);
		print_util_dbg_print_num(packet.param2,10);
		print_util_dbg_print_num(packet.param3,10);
		print_util_dbg_print_num(packet.param4,10);
		print_util_dbg_print_num(packet.param5,10);
		print_util_dbg_print_num(packet.param6,10);
		print_util_dbg_print_num(packet.param7,10);
		print_util_dbg_print(", command id:");
		print_util_dbg_print_num(packet.command,10);
		print_util_dbg_print(", confirmation:");
		print_util_dbg_print_num(packet.confirmation,10);
		print_util_dbg_print("\n");
		
		switch(packet.command) 
		{
			case MAV_CMD_NAV_WAYPOINT:
				/* Navigate to MISSION. 
				| Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)
				| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)
				| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
				| Desired yaw angle at MISSION (rotary wing)
				| Latitude
				| Longitude
				| Altitude
				| */
				print_util_dbg_print("Nav waypoint command, not implemented!\n");
				break;

			case MAV_CMD_NAV_LOITER_UNLIM:
				/* Loiter around this MISSION an unlimited amount of time 
				| Empty
				| Empty
				| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
				| Desired yaw angle.
				| Latitude
				| Longitude
				| Altitude
				| */
				print_util_dbg_print("Nav loiter unlim command, not implemented!\n");
				break;

			case MAV_CMD_NAV_LOITER_TURNS:
				/* Loiter around this MISSION for X turns 
				| Turns
				| Empty
				| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
				| Desired yaw angle.
				| Latitude
				| Longitude
				| Altitude
				| */
				print_util_dbg_print("Nav loiter turns command, not implemented!\n");
				break;

			case MAV_CMD_NAV_LOITER_TIME:
				/* Loiter around this MISSION for X seconds 
				| Seconds (decimal)
				| Empty
				| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
				| Desired yaw angle.
				| Latitude
				| Longitude
				| Altitude
				| */
				print_util_dbg_print("Nav loiter time command, not implemented!\n");
				break;

			case MAV_CMD_NAV_RETURN_TO_LAUNCH:
				/* Return to launch location 
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				print_util_dbg_print("Nav Return to launch command, not implemented!\n");
				break;

			case MAV_CMD_NAV_LAND:
				/* Land at location 
				| Empty
				| Empty
				| Empty
				| Desired yaw angle.
				| Latitude
				| Longitude
				| Altitude
				| */
				print_util_dbg_print("Command for automatic land, not implemented!\n");
				break;

			case MAV_CMD_NAV_TAKEOFF:
				/* Takeoff from ground / hand 
				| Minimum pitch (if airspeed sensor present), desired pitch without sensor
				| Empty
				| Empty
				| Yaw angle (if magnetometer present), ignored without magnetometer
				| Latitude
				| Longitude
				| Altitude
				| */
				centralData->in_the_air = true;
				print_util_dbg_print("Starting automatic take-off from button\n");
				break;

			case MAV_CMD_NAV_ROI:
				/* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. 
				| Region of interest mode. (see MAV_ROI enum)
				| MISSION index/ target ID. (see MAV_ROI enum)
				| ROI index (allows a vehicle to manage multiple ROI's)
				| Empty
				| x the location of the fixed ROI (see MAV_FRAME)
				| y
				| z
				| */
				print_util_dbg_print("Nav ROI command, not implemented!\n");
				break;

			case MAV_CMD_NAV_PATHPLANNING:
				/* Control autonomous path planning on the MAV. 
				| 0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
				| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
				| Empty
				| Yaw angle at goal, in compass degrees, [0..360]
				| Latitude/X of goal
				| Longitude/Y of goal
				| Altitude/Z of goal
				| */
				print_util_dbg_print("Nav pathplanning command, not implemented!\n");
				break;

			case MAV_CMD_NAV_LAST:
				/* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration 
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				print_util_dbg_print("Nav last command, not implemented!\n");
				break;

			case MAV_CMD_CONDITION_DELAY:
				/* Delay mission state machine. 
				| Delay in seconds (decimal)
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				print_util_dbg_print("Condition Delay command, not implemented!\n");
				break;

			case MAV_CMD_CONDITION_CHANGE_ALT:
				/* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. 
				| Descent / Ascend rate (m/s)
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Finish Altitude
				| */
				print_util_dbg_print("Condition change alt command, not implemented!\n");
				break;

			case MAV_CMD_CONDITION_DISTANCE:
				/* Delay mission state machine until within desired distance of next NAV point. 
				| Distance (meters)
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				print_util_dbg_print("Condition distance command, not implemented!\n");
				break;

			case MAV_CMD_CONDITION_YAW:
				/* Reach a certain target angle. 
				| target angle: [0-360], 0 is north
				| speed during yaw change:[deg per second]
				| direction: negative: counter clockwise, positive: clockwise [-1,1]
				| relative offset or absolute angle: [ 1,0]
				| Empty
				| Empty
				| Empty
				| */
				print_util_dbg_print("Condition yaw command, not implemented!\n");
				break;

			case MAV_CMD_CONDITION_LAST:
				/* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration 
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				print_util_dbg_print("Condition last command, not implemented!\n");
				break;

			case MAV_CMD_DO_SET_MODE:
				/* Set system mode. 
				| Mode, as defined by ENUM MAV_MODE
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				print_util_dbg_print("Do set mode command, not implemented!\n");
				break;

			case MAV_CMD_DO_JUMP:
				/* Jump to the desired command in the mission list.  Repeat this action only the specified number of times 
				| Sequence number
				| Repeat count
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				print_util_dbg_print("Do jump command, not implemented!\n");
				break;

			case MAV_CMD_DO_CHANGE_SPEED:
				/* Change speed and/or throttle set points. 
				| Speed type (0=Airspeed, 1=Ground Speed)
				| Speed  (m/s, -1 indicates no change)
				| Throttle  ( Percent, -1 indicates no change)
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				print_util_dbg_print("Do change speed command, not implemented!\n");
				break;

			case MAV_CMD_DO_SET_HOME:
				/* Changes the home location either to the current location or a specified location. 
				| Use current (1=use current location, 0=use specified location)
				| Empty
				| Empty
				| Empty
				| Latitude
				| Longitude
				| Altitude
				| */
				if (packet.param1 == 1)
				{
					// Set new home position to actual position
					print_util_dbg_print("Set new home location to actual position.\n");
					centralData->position_estimator.localPosition.origin = coord_conventions_local_to_global_position(centralData->position_estimator.localPosition);
					centralData->sim_model.localPosition.origin = centralData->position_estimator.localPosition.origin;
					
					print_util_dbg_print("New Home location: (");
					print_util_dbg_print_num(centralData->position_estimator.localPosition.origin.latitude * 10000000.0f,10);
					print_util_dbg_print(", ");
					print_util_dbg_print_num(centralData->position_estimator.localPosition.origin.longitude * 10000000.0f,10);
					print_util_dbg_print(", ");
					print_util_dbg_print_num(centralData->position_estimator.localPosition.origin.altitude * 1000.0f,10);
					print_util_dbg_print(")\n");
				}
				else
				{
					// Set new home position from msg
					print_util_dbg_print("Set new home location. \n");
					
					centralData->position_estimator.localPosition.origin.latitude = packet.param5;
					centralData->position_estimator.localPosition.origin.longitude = packet.param6;
					centralData->position_estimator.localPosition.origin.altitude = packet.param7;
					centralData->sim_model.localPosition.origin = centralData->position_estimator.localPosition.origin;
					
					print_util_dbg_print("New Home location: (");
					print_util_dbg_print_num(centralData->position_estimator.localPosition.origin.latitude * 10000000.0f,10);
					print_util_dbg_print(", ");
					print_util_dbg_print_num(centralData->position_estimator.localPosition.origin.longitude * 10000000.0f,10);
					print_util_dbg_print(", ");
					print_util_dbg_print_num(centralData->position_estimator.localPosition.origin.altitude * 1000.0f,10);
					print_util_dbg_print(")\n");
				}

				centralData->waypoint_set = false;
				break;

			case MAV_CMD_DO_SET_PARAMETER:
				/* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. 
				| Parameter number
				| Parameter value
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				|  */
				print_util_dbg_print("Set parameter command, not implemented!\n");
				break;

			case MAV_CMD_DO_SET_RELAY:
				/* Set a relay to a condition. 
				| Relay number
				| Setting (1=on, 0=off, others possible depending on system hardware)
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				|  */
				print_util_dbg_print("Set relay command, not implemented!\n");
				break;

			case MAV_CMD_DO_REPEAT_RELAY:
				/* Cycle a relay on and off for a desired number of cyles with a desired period. 
				| Relay number
				| Cycle count
				| Cycle time (seconds, decimal)
				| Empty
				| Empty
				| Empty
				| Empty
				|  */
				print_util_dbg_print("Repeat relay command, not implemented!\n");
				break;

			case MAV_CMD_DO_SET_SERVO:
				/* Set a servo to a desired PWM value. 
				| Servo number
				| PWM (microseconds, 1000 to 2000 typical)
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				|  */
				print_util_dbg_print("Set servo command, not implemented!\n");
				break;

			case MAV_CMD_DO_REPEAT_SERVO:
				/* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. 
				| Servo number
				| PWM (microseconds, 1000 to 2000 typical)
				| Cycle count
				| Cycle time (seconds)
				| Empty
				| Empty
				| Empty
				|  */
				print_util_dbg_print("Repeat servo command, not implemented!\n");
				break;

			case MAV_CMD_DO_CONTROL_VIDEO:
				/* Control onboard camera system. 
				| Camera ID (-1 for all)
				| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
				| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)
				| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
				| Empty
				| Empty
				| Empty
				|  */
				print_util_dbg_print("Control video command, not implemented!\n");
				break;

			case MAV_CMD_DO_LAST:
				/* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration 
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				|  */
				print_util_dbg_print("Do last command, not implemented!\n");
				break;

			case MAV_CMD_PREFLIGHT_CALIBRATION:
				/* Trigger calibration. This command will be only accepted if in pre-flight mode. 
				| Gyro calibration: 0: no, 1: yes
				| Magnetometer calibration: 0: no, 1: yes
				| Ground pressure: 0: no, 1: yes
				| Radio calibration: 0: no, 1: yes
				| Accelerometer calibration: 0: no, 1: yes
				| Empty
				| Empty
				|  */
				print_util_dbg_print("Preflight calibration command, not implemented!\n");
				break;

			case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
				/* Set sensor offsets. This command will be only accepted if in pre-flight mode. 
				| Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow
				| X axis offset (or generic dimension 1), in the sensor's raw units
				| Y axis offset (or generic dimension 2), in the sensor's raw units
				| Z axis offset (or generic dimension 3), in the sensor's raw units
				| Generic dimension 4, in the sensor's raw units
				| Generic dimension 5, in the sensor's raw units
				| Generic dimension 6, in the sensor's raw units
				|  */
				print_util_dbg_print("Set sensor offsets command, not implemented!\n");
				break;

			case MAV_CMD_PREFLIGHT_STORAGE:
				/* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. 
				| Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM
				| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM
				| Reserved
				| Reserved
				| Empty
				| Empty
				| Empty
				|  */
				
				// Onboard parameters storage
				if (packet.param1 == 0) 
				{
					// read parameters from flash
					print_util_dbg_print("Reading from flashc...\n");
					onboard_parameters_read_parameters_from_flashc();
				}
				else if (packet.param1 == 1) 
				{
					// write parameters to flash
					//print_util_dbg_print("No Writing to flashc\n");
					print_util_dbg_print("Writing to flashc\n");
					onboard_parameters_write_parameters_from_flashc();
				}
				
				// Mission parameters storage
				if (packet.param2 == 0) 
				{
					// read mission from flash
				}
				else if (packet.param2 == 1) 
				{
					// write mission to flash
				}
				break;

			case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
				/* Request the reboot or shutdown of system components. 
				| 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.
				| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.
				| Reserved
				| Reserved
				| Empty
				| Empty
				| Empty
				|  */
				print_util_dbg_print("Reboot/Shutdown command, not implemented!\n");
				break;

			case MAV_CMD_OVERRIDE_GOTO:
				/* Hold / continue the current action 
				| MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan
				| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position
				| MAV_FRAME coordinate frame of hold point
				| Desired yaw angle in degrees
				| Latitude / X position
				| Longitude / Y position
				| Altitude / Z position
				|  */
				print_util_dbg_print("Goto command, not implemented!\n");
				break;

			case MAV_CMD_MISSION_START:
				/* start running a mission 
				| first_item: the first mission item to run
				| last_item:  the last mission item to run (after this item is run, the mission ends)
				|  */
				print_util_dbg_print("Mission start command, not implemented!\n");
				break;

			case MAV_CMD_COMPONENT_ARM_DISARM:
				/* Arms / Disarms a component 
				| 1 to arm, 0 to disarm
				|  */
				print_util_dbg_print("Disarm command, not implemented!\n");
				break;

			case MAV_CMD_ENUM_END:
				/*  
				| 
				*/
				break;

		}
	}
	

	if((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER)		// TODO: this part needs comments
	{
		// print packet command and parameters for debug
		print_util_dbg_print("All vehicles parameters:");
		print_util_dbg_print_num(packet.param1,10);
		print_util_dbg_print_num(packet.param2,10);
		print_util_dbg_print_num(packet.param3,10);
		print_util_dbg_print_num(packet.param4,10);
		print_util_dbg_print_num(packet.param5,10);
		print_util_dbg_print_num(packet.param6,10);
		print_util_dbg_print_num(packet.param7,10);
		print_util_dbg_print(", command id:");
		print_util_dbg_print_num(packet.command,10);
		print_util_dbg_print(", confirmation:");
		print_util_dbg_print_num(packet.confirmation,10);
		print_util_dbg_print("\n");
		
		switch(packet.command) {
			case MAV_CMD_NAV_RETURN_TO_LAUNCH:
				/* Return to launch location 
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				print_util_dbg_print("All MAVs: Return to first waypoint. \n");
				waypoint_handler_set_current_waypoint_from_parameter(centralData->waypoint_list,centralData->number_of_waypoints,0);
				break;

			case MAV_CMD_NAV_LAND:
				/* Land at location 
				| Empty
				| Empty
				| Empty
				| Desired yaw angle.
				| Latitude
				| Longitude
				| Altitude
				| */
				print_util_dbg_print("All MAVs: Auto-landing");
				break;

			case MAV_CMD_MISSION_START:
				/* start running a mission 
				| first_item: the first mission item to run
				| last_item:  the last mission item to run (after this item is run, the mission ends)
				| */
				print_util_dbg_print("All vehicles: Navigating to next waypoint. \n");
				waypoint_handler_continueToNextWaypoint();
				break;

			case MAV_CMD_CONDITION_LAST:
				/*
				| */
				print_util_dbg_print("All MAVs: setting circle scenario!\n");
				waypoint_handler_set_circle_scenario(centralData->waypoint_list, &(centralData->number_of_waypoints), packet.param1, packet.param2);
				break;			
		}
	}
	
}

void mavlink_actions_init(void) {
	
	centralData = central_data_get_pointer_to_struct();
	mavlink_actions_add_onboard_parameters();
	
	/*	
	 * Use this to store or read or reset parameters on flash memory
	 *
	onboard_parameters_write_parameters_from_flashc();
	onboard_parameters_read_parameters_from_flashc();
	 *
	 */

	print_util_dbg_print("MAVlink actions initialiased\n");
}
