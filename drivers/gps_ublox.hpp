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
 * \file gps_ublox.hpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief Driver for UBLOX GPS
 *
 * \detail  Only one instance can be used
 *          TODO: remove global variables in gps_ublox.cpp
 *
 ******************************************************************************/


#ifndef GPS_UBLOX_HPP_
#define GPS_UBLOX_HPP_

#include "drivers/gps.hpp"

#include "hal/common/serial.hpp"


/**
 * \brief Driver for UBLOX GPS
 */
class Gps_ublox: public Gps
{
public:
    /**
     * \brief   Constructor
     *
     * \param   serial  Reference to serial peripheral
     */
    Gps_ublox(Serial& serial);


    /**
     * \brief   Main update function
     * \detail  Reads new values from sensor
     *
     * \return  Success
     */
    bool update(void);


    /**
     * \brief   Get last update time in microseconds
     *
     * \return  Update time
     */
    const float& last_update_us(void) const;


    /**
     * \brief   Get last position update time in microseconds
     *
     * \return  Update time
     */
    const float& last_position_update_us(void) const;


    /**
     * \brief   Get last velocity update time in microseconds
     *
     * \return  Update time
     */
    const float& last_velocity_update_us(void) const;


    /**
     * \brief   Get position in global frame
     *
     * \return  position
     */
    const global_position_t& position_gf(void) const;


    /**
     * \brief   Get horizontal position accuracy in m
     *
     * \return  accuracy
     */
    const float& horizontal_position_accuracy(void) const;


    /**
     * \brief   Get vertical position accuracy in m
     *
     * \return  accuracy
     */
    const float& vertical_position_accuracy(void) const;


    /**
     * \brief   Get velocity in global frame in m/s
     *
     * \return  3D velocity
     */
    const std::array<float, 3>& velocity_lf(void) const;


    /**
     * \brief   Get velocity accuracy in m/s
     *
     * \return  velocity accuracy
     */
    const float& velocity_accuracy(void) const;


    /**
     * \brief   Get heading in degrees
     *
     * \return  heading
     */
    const float& heading(void) const;


    /**
     * \brief   Get heading accuracy in degrees
     *
     * \return  accuracy
     */
    const float& heading_accuracy(void) const;


    /**
     * \brief   Get the number of satellites
     *
     * \return  Value
     */
    const uint8_t& num_sats(void) const;


    /**
     * \brief   Indicates whether fix are received
     *
     * \return  Value
     */
    const gps_fix_t& fix(void) const;


    /**
     * \brief   Indicates whether the measurements can be trusted
     *
     * \return  Value
     */
    const bool& healthy(void) const;


    /**
     * \brief   Start configuration
     *
     * \detail  The update function has to be called in order to proceed with
     *          the configuration
     */
    void configure(void);


private:
    Serial&             serial_;                            ///< Reference to Serial peripheral

    float               last_update_us_;                    ///< Last update time in microseconds
    float               last_position_update_us_;           ///< Last time position was updated in microseconds
    float               last_velocity_update_us_;           ///< Last time velocity was updated in microseconds
    global_position_t   position_gf_;                       ///< Global position
    float               horizontal_position_accuracy_;      ///< Accuracy of position on horizontal plane in m
    float               vertical_position_accuracy_;        ///< Accuracy of position on vertical axis in m
    std::array<float, 3> velocity_lf_;                      ///< 3D Velocity in lobal frame in m/s
    float               velocity_accuracy_;                 ///< Accuracy of velocity in m/s
    float               heading_;                           ///< Heading in degrees
    float               heading_accuracy_;                  ///< Accuracy of heading in degrees
    uint8_t             num_sats_;                          ///< Number of visible satelites
    gps_fix_t           fix_;                               ///< Indicates whether a fix was acquired
    bool                healthy_;                           ///< Indicates whether the measurements can be trusted
};



#endif /* GPS_UBLOX_HPP_ */