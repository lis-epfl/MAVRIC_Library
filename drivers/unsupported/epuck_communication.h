/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file epuck_communication.h
 *
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *
 * \brief   This file configures the epuck UART communication
 *          and send remote scaled messages to the epuck to be used to drive its wheels
 *
 ******************************************************************************/
#ifndef EPUCK_COMMUNICATION_H_
#define EPUCK_COMMUNICATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_stream.h"
#include "buffer.h"
#include "remote.h"

/**
 * \brief Defines the state machine structure
 */
typedef struct
{
    mavlink_stream_t    mavlink_stream;     ///< Mavlink interface using streams
    byte_stream_t       uart_stream_in;     ///< stream from Epuck
    byte_stream_t       uart_stream_out;    ///< stream towards Epuck
    buffer_t            uart_buffer_in;     ///< buffer for messages received from Epuck
    buffer_t            uart_buffer_out;    ///< buffer for messages to sent towards Epuck

    const remote_t*     remote;             ///< Pointer to the remote structure
} epuck_communication_t;

/**
 * \brief                       Initialize the epuck comm module
 *
 * \param epuck_communication   Pointer to the epuck communication structure
 * \param remote                Pointer to the remote structure
 * \param UID                   UART ID from UART0 to UART4
 * \param usart_conf_epuck      Uart configuration to talk to the epuck
 */
void epuck_communication_init(epuck_communication_t* epuck_communication, const remote_t* remote, int32_t UID, usart_config_t usart_conf_epuck);

/**
 * \brief   Updates the Epuck communication
 *
 * \param   epuck_communication         Pointer to the epuck_communication structure
 *
 * \return Returns the result of the task
 */
bool epuck_communication_update(epuck_communication_t* epuck_communication);


#ifdef __cplusplus
}
#endif

#endif //EPUCK_COMMUNICATION_H_