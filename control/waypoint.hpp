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

#include "control/dubin.hpp"
#include "communication/mavlink_communication.hpp"
#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "util/coord_conventions.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Waypoint
{
public:
    /**
     * \brief   Creates a blank waypoint
     */
    Waypoint();

    /**
     * \brief   Initialize the waypoint handler
     *
     * \param   packet                  The received packet for creating a waypoint
     */
    Waypoint(mavlink_mission_item_t& packet);

    /**
     * \brief   Initialize the waypoint handler
     *
     * \param   frame               The reference frame of the waypoint
     * \param   command             The MAV_CMD_NAV id of the waypoint
     * \param   autocontinue        Flag to tell whether the vehicle should auto continue to the next waypoint once it reaches the current waypoint
     * \param   param1              Parameter depending on the MAV_CMD_NAV id
     * \param   param2              Parameter depending on the MAV_CMD_NAV id
     * \param   param3              Parameter depending on the MAV_CMD_NAV id
     * \param   param4              Parameter depending on the MAV_CMD_NAV id
     * \param   param5              Parameter depending on the MAV_CMD_NAV id (usually x/latitude)
     * \param   param6              Parameter depending on the MAV_CMD_NAV id (usually y/longitude)
     * \param   param7              Parameter depending on the MAV_CMD_NAV id (usually z/altitude)
     */
    Waypoint(   uint8_t frame,
                uint16_t command,
                uint8_t autocontinue,
                float param1,
                float param2,
                float param3,
                float param4,
                float param5,
                float param6,
                float param7);

    /**
     * \brief   Sends a given waypoint via a MAVLink message
     *
     * \param   mavlink_stream          The mavlink stream to send the message through
     * \param   sysid                   The system ID
     * \param   msg                     The pointer to the received MAVLink message structure asking for a waypoint
     * \param   seq                     The sequence number of the packet
     * \param   current                 States if the waypoint is the current waypoint (If is the waypoint we are heading towards and are current en route to waypoint)
     */
    void send(const Mavlink_stream& mavlink_stream, uint32_t sysid, mavlink_message_t* msg, uint16_t seq, uint8_t current);

    /**
     * \brief   Gets the frame of the waypoint
     *
     * \return  frame
     */
    uint8_t frame() const;

    /**
     * \brief   Sets the frame
     *
     * \param   frame
     */
    void set_frame(uint8_t frame);

    /**
     * \brief   Gets the command of the waypoint
     *
     * \return  command
     */
    uint16_t command() const;

    /**
     * \brief   Sets the command
     *
     * \param   command
     */
    void set_command(uint16_t command);

    /**
     * \brief   Gets the autocontinue state of the waypoint
     *
     * \return  autocontinue
     */
    uint8_t autocontinue() const;

    /**
     * \brief   Sets the autocontinue
     *
     * \param   autocontinue
     */
    void set_autocontinue(uint8_t autocontinue);

    /**
     * \brief   Gets param1 of the waypoint
     *
     * \return  param1
     */
    float param1() const;

    /**
     * \brief   Sets param1
     *
     * \param   param1
     */
    void set_param1(float param1);

    /**
     * \brief   Gets param2 of the waypoint
     *
     * \return  param2
     */
    float param2() const;

    /**
     * \brief   Sets param2
     *
     * \param   param2
     */
    void set_param2(float param2);

    /**
     * \brief   Gets param3 of the waypoint
     *
     * \return  param3
     */
    float param3() const;

    /**
     * \brief   Sets param3
     *
     * \param   param3
     */
    void set_param3(float param3);

    /**
     * \brief   Gets param4 of the waypoint
     *
     * \return  param4
     */
    float param4() const;

    /**
     * \brief   Sets param4
     *
     * \param   param4
     */
    void set_param4(float param4);

    /**
     * \brief   Gets param5 of the waypoint
     *
     * \return  param5
     */
    float param5() const;

    /**
     * \brief   Sets param5
     *
     * \param   param5
     */
    void set_param5(float param5);

    /**
     * \brief   Gets param6 of the waypoint
     *
     * \return  param6
     */
    float param6() const;

    /**
     * \brief   Sets param6
     *
     * \param   param6
     */
    void set_param6(float param6);

    /**
     * \brief   Gets param7 of the waypoint
     *
     * \return  param7
     */
    float param7() const;

    /**
     * \brief   Sets param7
     *
     * \param   param7
     */
    void set_param7(float param7);

    /**
     * \brief   Gets the waypoint in local coordinates
     *
     * \return  Local waypoint position
     */
    local_position_t local_pos() const;

    /**
     * \brief   Sets the waypoint in local coordinates
     *
     * \param   local_pos       The local position of the waypoint
     */
    //void set_local_pos(local_position_t local_pos);

    /**
     * \brief   Sets the waypoint in local coordinates
     *
     * \param   local_pos       The locaiton position of the waypoint
     * \param   heading         The desired heading of the waypoint
     */
    //void set_local_pos(local_position_t local_pos, float heading);

protected:
    uint8_t frame_;                                             ///< The reference frame of the waypoint
    uint16_t command_;                                          ///< The MAV_CMD_NAV id of the waypoint
    uint8_t autocontinue_;                                      ///< Flag to tell whether the vehicle should auto continue to the next waypoint once it reaches the current waypoint
    float param1_;                                              ///< Parameter depending on the MAV_CMD_NAV id
    float param2_;                                              ///< Parameter depending on the MAV_CMD_NAV id
    float param3_;                                              ///< Parameter depending on the MAV_CMD_NAV id
    float param4_;                                              ///< Parameter depending on the MAV_CMD_NAV id
    float param5_;                                              ///< Parameter depending on the MAV_CMD_NAV id
    float param6_;                                              ///< Parameter depending on the MAV_CMD_NAV id
    float param7_;                                              ///< Parameter depending on the MAV_CMD_NAV id

    /**
     * \brief   Determines the global position of the waypoint based on the frame
     * and x, y, z, coordinates
     *
     * \param frame     The MAV_FRAME of the x, y, z coordinates
     * \param x         The x coordinate
     * \param y         The y coordinate
     * \param z         The z coordinate
     * \param origin    The origin of the local frame (if needed)
     *
     * \return  Global frame position
     */
    static global_position_t get_global_position(uint8_t frame, double x, double y, double z, global_position_t origin);

    /**
     * \brief   Determines the position of the waypoint based in the
     * desired frame
     *
     * If position estimation is NULL, returns (0, 0, 0)
     *
     * \param x     x coordinate in desired frame (output)
     * \param y     y coordinate in desired frame (output)
     * \param z     z coordinate in desired frame (output)
     * \param frame The desired frame
     */
    void get_waypoint_parameters(double& x, double& y, double& z, uint8_t frame) const;
};





#endif // WAYPOINT__
