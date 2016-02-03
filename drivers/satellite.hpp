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
 * \file satellite.hpp
 *
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *
 * \brief This declare the global satellite struct
 * enable usage of different satellite receiver (ie. spektrum, emulated...)
 *
 ******************************************************************************/

#ifndef SATELLITE_HPP_
#define SATELLITE_HPP_

#include <stdint.h>
#include <stdbool.h>

/**
 * \brief Radio protocols
 */
typedef enum
{
    RADIO_PROTOCOL_DSM2_10BITS  = 0,
    RADIO_PROTOCOL_DSM2_11BITS  = 1,
    RADIO_PROTOCOL_DSMX         = 2,
    RADIO_PROTOCOL_UNKNOWN      = 3,
} radio_protocol_t;


class Satellite
{
public:

    /**
    * \brief    Virtual function to intialize a satellite receiver
    */
    virtual bool init(void) = 0;


    /**
    * \brief    Virtual function to bind a satellite with a remote
    *
    * \param    radio_protocol  Define in which protocol the remote has to be binded
    */
    virtual void bind(const radio_protocol_t radio_protocol) = 0;


    /**
    * \brief    Return a channels' value
    *
    * \param    channel_number      The channel ID
    *
    * \return   Value for channel channel_number
    */
    virtual int16_t channel(const uint8_t channel_number) const = 0;


    /**
    * \brief    Return the last update time in microseconds
    *
    * \return   Last update time
    */
    virtual uint32_t last_update(void) const = 0;


    /**
    * \brief    Return the time difference between the last 2 updates in microseconds
    *
    * \return   dt
    */
    virtual uint32_t dt(void) const = 0;
};


#endif //SATELLITE_HPP_
