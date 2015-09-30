/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file dc_motor_ctrl.h
 *
 * This file configures the dc_motor_ctrl UART communication
 */

#ifndef DC_MOTOR_CTRL_H_
#define DC_MOTOR_CTRL_H_

#include "streams.h"
#include "buffer.h"
#include "mavlink_stream.h"
#include "scheduler.h"

/**
 * \brief structure of the i2cxl_sonar module
*/
typedef struct
{
	buffer_t dc_motor_ctrl_in_buffer;			///< The dc_motor_ctrl incoming buffer
	byte_stream_t dc_motor_ctrl_out_stream;		///< The dc_motor_ctrl outgoing byte stream
	byte_stream_t dc_motor_ctrl_in_stream;		///< The dc_motor_ctrl incoming byte stream
	float wingrons_angle[2];                    ///< Angles wanted for the wingrons dc_motors
	float wingrons_speed[2];					///< Wanted speed for the wingrons dc_motors
	const mavlink_stream_t* mavlink_stream;		///< Pointer to mavlink stream
} daler_dc_motor_ctrl_t;


/**
 * \brief Initialize the dc_motor_ctrl module
 *
 * \param dc_motor_ctrl pointer to DC motor controller structure
 * \param UID uart device number
 * 
 * \return 	success
 */
bool daler_dc_motor_ctrl_init(daler_dc_motor_ctrl_t* dc_motor_ctrl, int32_t UID);


/**
 * \brief Update the dc_motor_ctrl module
 *
 * \param dc_motor_ctrl pointer to DC motor controller structure
 * 
 * \return success
 */
bool daler_dc_motor_ctrl_update(daler_dc_motor_ctrl_t* dc_motor );//, const float wingrons[2] );

#endif /* DC_MOTOR_CTRL_H_ */