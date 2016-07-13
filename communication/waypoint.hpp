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
 * \file waypoint.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The waypoint class
 *
 ******************************************************************************/


#ifndef WAYPOINT__
#define WAYPOINT__

#include "communication/mavlink_communication.hpp"
#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_message_handler.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Waypoint
{
public:
    /**
     * \brief   Initialize the waypoint handler
     *
     * \param   mavlink_stream          The reference to the MAVLink stream structure
     * \param   packet                  The received packet for creating a waypoint
     */
    Waypoint(const Mavlink_stream& mavlink_stream, mavlink_mission_item_t packet);

    /**
     * \brief   Initialize the waypoint handler
     *
     * \param   mavlink_stream      The reference to the MAVLink stream structure
     * \param   frame               The reference frame of the waypoint
     * \param   command             The MAV_CMD_NAV id of the waypoint
     * \param   autocontinue        Flag to tell whether the vehicle should auto continue to the next waypoint once it reaches the current waypoint
     * \param   param1              Parameter depending on the MAV_CMD_NAV id
     * \param   param2              Parameter depending on the MAV_CMD_NAV id
     * \param   param3              Parameter depending on the MAV_CMD_NAV id
     * \param   param4              Parameter depending on the MAV_CMD_NAV id
     * \param   x                   The value on the x axis (depends on the reference frame)
     * \param   y                   The value on the y axis (depends on the reference frame)
     * \param   z                   The value on the z axis (depends on the reference frame)
     */
    Waypoint(   const Mavlink_stream& mavlink_stream_,
                uint8_t frame,
                uint16_t command,
                uint8_t autocontinue,
                float param1,
                float param2,
                float param3,
                float param4,
                float x,
                float y,
                float z)

    /**
     * \brief   Calculates the information required for waypoint local structure
     *
     * \param   origin                  The coordinates (latitude, longitude and altitude in global frame) of the local frame's origin
     * \param   dubin_state             The pointer to the Dubin state
     */
    void calculate_waypoint_local_structure(global_position_t origin, dubin_state_t* dubin_state);

    /**
     * \brief   Sends a given waypoint via a MAVLink message
     *
     * \param   sysid                   The system ID
     * \param   msg                     The pointer to the received MAVLink message structure asking for a waypoint
     * \param   seq                     The sequence number of the packet
     * \param   current                 States if the waypoint is the current waypoint (If is the waypoint we are heading towards and are current en route to waypoint)
     */
    void send(uint32_t sysid, mavlink_message_t* msg, uint16_t seq, uint8_t current);

    /**
     * \brief   Gets the waypoint in local coordinates
     *
     * \return  Local waypoint position
     */
    local_position_t local_pos() const;

    /**
     * \brief   Sets the waypoint in local coordinates
     *
     * NOTE: Does not set anything any other field in the class
     * TODO: Set other fields in the class
     */
    void set_local_pos(local_position_t local_pos);

    /**
     * \brief   Gets the radius
     *
     * \return  radius
     */
    float radius() const;

    /**
     * \brief   Gets a reference to the dubin structure
     *
     * \return  Dubin structure for the waypoint
     */
    dubin_t& dubin();

protected:
    uint8_t frame_;                                              ///< The reference frame of the waypoint
    uint16_t command_;                                           ///< The MAV_CMD_NAV id of the waypoint
    uint8_t autocontinue_;                                       ///< Flag to tell whether the vehicle should auto continue to the next waypoint once it reaches the current waypoint
    float param1_;                                               ///< Parameter depending on the MAV_CMD_NAV id
    float param2_;                                               ///< Parameter depending on the MAV_CMD_NAV id
    float param3_;                                               ///< Parameter depending on the MAV_CMD_NAV id
    float param4_;                                               ///< Parameter depending on the MAV_CMD_NAV id
    double x_;                                                   ///< The value on the x axis (depends on the reference frame)
    double y_;                                                   ///< The value on the y axis (depends on the reference frame)
    double z_;                                                   ///< The value on the z axis (depends on the reference frame)

    const Mavlink_stream& mavlink_stream_;                      ///< The mavlink stream

    // These come from Navigation::waypoint_local_struct_t
    // TODO: Get rid of for more logical structure
    local_position_t local_pos_;                                  ///< The local coordinates of the waypoint, was called Navigation::waypoint_local_struct_t::waypoint
    float radius_;                                               ///< The radius to turn around the waypoint, positive value for clockwise orbit, negative value for counter-clockwise orbit
    float loiter_time_;                                          ///< The loiter time at the waypoint
    dubin_t dubin_;                                              ///< The Dubin structure
};





#endif // WAYPOINT__
