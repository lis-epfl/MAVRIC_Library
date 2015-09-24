/*******************************************************************************
 * Copyright (c) 2009-2015, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file flow.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Driver for optic flow sensors
 *
 ******************************************************************************/

#ifndef FLOW_H_
#define FLOW_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_stream.h"
#include "buffer.h"
#include "uart_int.h"

#include <stdint.h>


/**
 * \brief 	Array of 2-D optic flow vectors
 */
typedef union
{
	struct 
	{
		int16_t x[125];		///< Horizontal component
		int16_t y[125];		///< Vertical component
	};
	uint8_t data[500];		///< Raw access to data
} flow_data_t;


/**
 * \brief	State of encapsulated data transfer
 */
typedef enum
{
	FLOW_NO_HANDSHAKE       = 0,
	FLOW_HANDSHAKE_DATA     = 1,
	FLOW_HANDSHAKE_METADATA = 2,
} flow_handshake_state_t;


/**
 * \brief 	Data structure for Flow
 */
typedef struct
{
	mavlink_stream_t 	mavlink_stream;		///< Mavlink interface using streams
	byte_stream_t		uart_stream_in;		///< stream from Epuck
	byte_stream_t		uart_stream_out;	///< stream towards Epuck
	buffer_t		uart_buffer_in;		///< buffer for messages received from Epuck
	buffer_t		uart_buffer_out;	///< buffer for messages to sent towards Epuck

	uint8_t 	of_count;			///< Number of optic flow vectors
	flow_data_t 	of;				///< Optic flow vectors
	flow_data_t 	of_loc;				///< Location of optic flow vectors

	flow_handshake_state_t  handshake_state; 	///< Indicates the current reception state for encapsulated data
	uint16_t 		n_packets;		///< Number of encapsulated data packets expected
	uint32_t 		size_data; 		///< Total size of data to receive (in bytes)

	uint32_t last_update_us;			///< Last update time in microseconds

	float tmp_flow_x;		///< Tmp debug data
	float tmp_flow_y;		///< Tmp debug data
	float tmp_flow_comp_m_x;	///< Tmp debug data
	float tmp_flow_comp_m_y;	///< Tmp debug data
} flow_t;


/**
 * \brief Init function
 * 
 * \param flow 		Pointer to flow structure
 * \param UID  		Uart ID
 * \param usart_conf 	Uart config
 */
void flow_init(flow_t* flow, int32_t UID, usart_config_t usart_conf);


/**
 * \brief Update function
 * 
 * \param flow 		Pointer to flow structure
 */
void flow_update(flow_t* flow);


#ifdef __cplusplus
}
#endif

#endif /* FLOW_H_ */
