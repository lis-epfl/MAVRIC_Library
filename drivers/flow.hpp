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
 * \file flow.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Driver for optic flow sensors
 *
 ******************************************************************************/

#ifndef FLOW_H_
#define FLOW_H_

#include "communication/mavlink_stream.hpp"
#include <stdint.h>
#include "hal/common/serial.hpp"

float const filter_constant = (1./250.)/((1./100.) + (1./500.) );
/**
 * \brief   Array of 2-D optic flow vectors
 */
typedef union
{
    struct
    {
        int16_t x[125];     ///< Horizontal component
        int16_t y[125];     ///< Vertical component
    };
    uint8_t data[500];      ///< Raw access to data
} flow_data_t;


/**
 * \brief   State of encapsulated data transfer
 */
typedef enum
{
    FLOW_NO_HANDSHAKE       = 0,    ///< No handshake received
    FLOW_HANDSHAKE_DATA     = 1,    ///< Normal data will be received
    FLOW_HANDSHAKE_METADATA = 2,    ///< Metadata will be received
} flow_handshake_state_t;


/**
 * \brief   Data structure for Flow
 */
typedef struct
{
    Serial*             uart;               ///< Serial device
    mavlink_stream_t    mavlink_stream;     ///< Mavlink interface using streams

    uint8_t     of_count;   ///< Number of optic flow vectors
    flow_data_t of;         ///< Optic flow vectors
    flow_data_t of_tmp;     ///< Temporary optic flow vectors
    flow_data_t of_loc;     ///< Location of optic flow vectors
    flow_data_t of_loc_tmp; ///< Temporary location of optic flow vectors
    uint16_t    n_packets;  ///< Number of encapsulated data packets expected
    uint32_t    size_data;  ///< Total size of data to receive (in bytes)

    flow_handshake_state_t  handshake_state;    ///< Indicates the current reception state for encapsulated data
    uint32_t last_update_us;                    ///< Last update time in microseconds
} flow_t;


/**
 * \brief Init function
 *
 * \param flow  Pointer to flow structure
 * \param uart  Pointer to serial peripheral
 *
 * \return      Success
 */
bool flow_init(flow_t* flow, Serial* uart);


/**
 * \brief Update function
 *
 * \param flow      Pointer to flow structure
 *
 * \return      Success
 */
bool flow_update(flow_t* flow);


#endif /* FLOW_HPP_ */
