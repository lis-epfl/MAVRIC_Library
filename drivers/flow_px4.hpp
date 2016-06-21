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

#ifndef FLOW_PX4_HPP_
#define FLOW_PX4_HPP_

#include "drivers/flow.hpp"
#include "communication/mavlink_stream.hpp"
#include <stdint.h>
#include "hal/common/serial.hpp"

/**
 * \brief   Array of 2-D optic flow vectors
 */
typedef union
{
    struct
    {
        int16_t x[70];     ///< Horizontal component
        int16_t y[70];     ///< Vertical component
    };
    uint8_t data[280];      ///< Raw access to data
} flow_px4_data_t;


/**
 * \brief   State of encapsulated data transfer
 */
typedef enum
{
    FLOW_NO_HANDSHAKE       = 0,    ///< No handshake received
    FLOW_HANDSHAKE_DATA     = 1,    ///< Normal data will be received
    FLOW_HANDSHAKE_METADATA = 2,    ///< Metadata will be received
} flow_px4_handshake_state_t;


/**
 * \brief   Driver for PX4Flow
 */
class  Flow_px4: public Flow
{
public:
    /**
    * \brief    Init function
    * \param    uart    Pointer to serial peripheral
    * \return   Success
    */
    Flow_px4(Serial& uart);

    /**
    * \brief    Update function
    * \return   Success
    */
    bool update(void);

private:
    Serial&             uart_;               ///< Serial device
    Mavlink_stream      mavlink_stream_;     ///< Mavlink interface using streams

    flow_px4_data_t of_tmp_;        ///< Temporary optic flow vectors
    flow_px4_data_t of_loc_tmp_;    ///< Temporary location of optic flow vectors
    uint16_t    packet_count_;  ///< Number of encapsulated data packets expected
    uint32_t    byte_count_;    ///< Total size of data to receive (in bytes)

    flow_px4_handshake_state_t  handshake_state;    ///< Indicates the current reception state for encapsulated data
};




#endif /* FLOW_PX4_HPP_ */
