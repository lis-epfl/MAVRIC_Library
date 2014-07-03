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
 * \file mavlink_action.h
 *
 * Definition of the tasks executed on the autopilot
 */ 


#ifndef MAVLINK_ACTIONS_H_
#define MAVLINK_ACTIONS_H_

#include "mavlink_stream.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief     Initialisation of the module mavlink actions
 */
void mavlink_actions_init(void);


/**
 * \brief     Add all onboard parameters to the parameter list
 */
void mavlink_actions_add_onboard_parameters(void);


/**
 * \brief     		Handle project-specific mavlink messages
 * \details 		A mavlink message is first handled in Library/mavlink_stream, 
 * 					it is forwarded to this function if it does not match any 
 * 					general messages
 * 
 * \param 	rec 	Received mavlink message 
 */
void mavlink_actions_handle_specific_messages(Mavlink_Received_t* rec);


/**
 * \brief     		Handle long mavlink messages (ie. MAV_CMD_XXX messages)
 * 
 * \param 	rec 	Received message
 */
void mavlink_actions_receive_message_long(Mavlink_Received_t* rec);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_ACTIONS_H_ */