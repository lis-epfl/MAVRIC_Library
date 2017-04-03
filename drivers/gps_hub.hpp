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
 * \file gps_hub.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Hub for GPS redundancy
 *
 ******************************************************************************/


#ifndef GPS_HUB_HPP_
#define GPS_HUB_HPP_

#include "drivers/gps.hpp"


/**
 * \brief       Hub for GPS redundancy
 *
 * \tparam  N   Number of GPS connected to hub
 */
template<uint32_t N>
class Gps_hub: public Gps
{
public:
    static_assert(N >= 1, "Need at least one GPS");

    /**
     * \brief Constructor
     */
    Gps_hub(std::array<Gps*, N> gps_list):
        gps_list_(gps_list),
        current_gps_(gps_list_[0])
    {};

    /**
     * \brief   Main update function
     * \detail  Reads new values from sensor
     *
     * \return  Success
     */
    bool update(void)
    {
        bool ret = false;

        // Update all gps
        for (size_t i = 0; i < N; i++)
        {
            ret |= gps_list_[i]->update();
        }

        // Choose which one is best
        for (size_t i = 0; i < N; i++)
        {
            if (gps_list_[i]->healthy())
            {
                if ((gps_list_[i]->fix() > current_gps_->fix())
                || ((gps_list_[i]->fix() == current_gps_->fix()) && (gps_list_[i]->num_sats() > current_gps_->num_sats())))
                {
                    current_gps_ = gps_list_[i];
                }
            }
        }

        return ret;
    };

    /**
     * \brief   Initializes the gps
     *
     * \return  Success
     */
    bool init(void)
    {
        bool ret = false;

        // Init all gps
        for (size_t i = 0; i < N; i++)
        {
            ret |= gps_list_[i]->init();
        }

        return ret;
    };


    /**
     * \brief   (re)Configure the GPS
     */
    void configure(void)
    {
        // Configure all gps
        for (size_t i = 0; i < N; i++)
        {
            gps_list_[i]->configure();
        }
    };


    /**
     * \brief   Get last update time in microseconds
     *
     * \return  Update time
     */
    time_us_t last_update_us(void) const
    {
        return current_gps_->last_update_us();
    };


    /**
     * \brief   Get last position update time in microseconds
     *
     * \return  Update time
     */
    time_us_t last_position_update_us(void) const
    {
        return current_gps_->last_position_update_us();
    };


    /**
     * \brief   Get last velocity update time in microseconds
     *
     * \return  Update time
     */
    time_us_t last_velocity_update_us(void) const
    {
        return current_gps_->last_velocity_update_us();
    };


    /**
     * \brief   Get position in global frame
     *
     * \return  position
     */
    global_position_t position_gf(void) const
    {
        return current_gps_->position_gf();
    };


    /**
     * \brief   Get horizontal position accuracy in m
     *
     * \return  accuracy
     */
    float horizontal_position_accuracy(void) const
    {
        return current_gps_->horizontal_position_accuracy();
    };


    /**
     * \brief   Get vertical position accuracy in m
     *
     * \return  accuracy
     */
    float vertical_position_accuracy(void) const
    {
        return current_gps_->vertical_position_accuracy();
    };


    /**
     * \brief   Get velocity in local frame in m/s
     *
     * \return  3D velocity
     */
    std::array<float, 3> velocity_lf(void) const
    {
        return current_gps_->velocity_lf();
    };


    /**
     * \brief   Get velocity accuracy in m/s
     *
     * \return  velocity accuracy
     */
    float velocity_accuracy(void) const
    {
        return current_gps_->velocity_accuracy();
    };


    /**
     * \brief   Get heading in degrees
     *
     * \return  heading
     */
    float heading(void) const
    {
        return current_gps_->heading();
    };


    /**
     * \brief   Get heading accuracy in degrees
     *
     * \return  accuracy
     */
    float heading_accuracy(void) const
    {
        return current_gps_->heading_accuracy();
    };


    /**
     * \brief   Get the number of satellites
     *
     * \return  Value
     */
    uint8_t num_sats(void) const
    {
        return current_gps_->num_sats();
    };


    /**
     * \brief   Indicates whether fix are received
     *
     * \return  Value
     */
    gps_fix_t fix(void) const
    {
        return current_gps_->fix();
    };


    /**
     * \brief   Indicates whether the measurements can be trusted
     *
     * \return  Value
     */
    bool healthy(void) const
    {
        return current_gps_->healthy();
    };

protected:
    std::array<Gps*, N> gps_list_;
    Gps*                current_gps_;
};


#endif /* GPS_HUB_HPP_ */
