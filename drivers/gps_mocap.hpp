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
 * \file gps_mocap.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Expose data received from motion capture system as GPS data
 *
 ******************************************************************************/


#ifndef GPS_MOCAP_HPP_
#define GPS_MOCAP_HPP_


#include <array>

extern "C"
{
#include "communication/mavlink_message_handler.hpp"
#include "util/coord_conventions.h"
}


/**
 * \brief   Configuration structure
 */
typedef struct
{
    global_position_t origin;
} gps_mocap_conf_t;


/**
 * \brief   Default configuration structure
 * 
 * \return  config
 */
static inline gps_mocap_conf_t gps_mocap_default_config();


/**
 * \brief Class to expose data received from motion capture system as GPS data
 * 
 * \detail The mocap data is received via mavlink messages of type ATT_POS_MOCAP
 */
class Gps_mocap
{
public:
    /**
     * \brief Constructor
     */
    Gps_mocap(mavlink_message_handler_t& message_handler, gps_mocap_conf_t config = gps_mocap_default_config() );


    /**
     * \brief  Initializes callback to mavlink messages
     * 
     * \return Success
     */
    bool init(void);


    /**
     * \brief   Main update function
     * \detail  Reads new values from sensor
     *
     * \return  Success
     */
    bool update(void);


    /**
     * \brief   Configure the GPS
     */
    void configure(void);


    /**
     * \brief   Get last update time in microseconds
     *
     * \return  Update time
     */
    const float last_update_us(void) const;


    /**
     * \brief   Get last position update time in microseconds
     *
     * \return  Update time
     */
    const float last_position_update_us(void) const;


    /**
     * \brief   Get last velocity update time in microseconds
     *
     * \return  Update time
     */
    const float last_velocity_update_us(void) const;


    /**
     * \brief   Get position in global frame
     *
     * \return  position
     */
    const global_position_t position_gf(void) const;


    /**
     * \brief   Get horizontal position accuracy in m
     *
     * \return  accuracy
     */
    const float horizontal_position_accuracy(void) const;


    /**
     * \brief   Get vertical position accuracy in m
     *
     * \return  accuracy
     */
    const float vertical_position_accuracy(void) const;


    /**
     * \brief   Get velocity in local frame in m/s
     *
     * \return  3D velocity
     */
    const std::array<float, 3> velocity_lf(void) const;


    /**
     * \brief   Get velocity accuracy in m/s
     *
     * \return  velocity accuracy
     */
    const float velocity_accuracy(void) const;


    /**
     * \brief   Get heading in degrees
     *
     * \return  heading
     */
    const float heading(void) const;


    /**
     * \brief   Get heading accuracy in degrees
     *
     * \return  accuracy
     */
    const float heading_accuracy(void) const;


    /**
     * \brief   Get the number of satellites
     *
     * \return  Value
     */
    const uint8_t num_sats(void) const;


    /**
     * \brief   Indicates whether fix are received
     *
     * \return  Value
     */
    const bool fix(void) const;


    /**
     * \brief   Indicates whether the measurements can be trusted
     *
     * \return  Value
     */
    const bool healthy(void) const;


private:
    mavlink_message_handler_t& message_handler_;       ///< Reference to message handler for callbacks
    gps_mocap_conf_t config_;                          ///< Configuration

    bool is_init;                                      ///< Flag to prevent adding message callbacks several times
};


/**
 * \brief   Default configuration structure
 * 
 * \return  config
 */
static inline gps_mocap_conf_t gps_mocap_default_config()
{
    gps_mocap_conf_t conf = {};

    // EPFL esplanade
    conf.origin.longitude = 46.51852236174565f;
    conf.origin.latitude  = 6.566044801857777f;
    conf.origin.altitude  = 400.0f;
    conf.origin.heading   = 0.0f;

    return conf;
};


#endif /* GPS_MOCAP_HPP_ */