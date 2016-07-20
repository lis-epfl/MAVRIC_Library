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
#include "sensing/position_estimation.hpp"
#include "util/coord_conventions.h"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Waypoint
{
public:
    /**
     * \brief   Creates a blank waypoint
     *
     * SHOULD ONLY BE USED FOR CONSTRUCTORS DONE BY COMPILER FOR TEMPORARY
     * VARIABLES. USE ONE OF THE OTHER CONSTRUCTORS
     */
    Waypoint();

    /**
     * \brief   Creates a blank waypoint
     *
     * \param   position_estimation     The reference to the position estimation
     */
    Waypoint(const Position_estimation* position_estimation);

    /**
     * \brief   Initialize the waypoint handler
     *
     * \param   position_estimation     The reference to the position estimation
     * \param   packet                  The received packet for creating a waypoint
     */
    Waypoint(const Position_estimation* position_estimation, mavlink_mission_item_t& packet);

    /**
     * \brief   Initialize the waypoint handler
     *
     * \param   position_estimation The reference to the position estimation
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
    Waypoint(   const Position_estimation* position_estimation,
                uint8_t frame,
                uint16_t command,
                uint8_t autocontinue,
                float param1,
                float param2,
                float param3,
                float param4,
                float x,
                float y,
                float z);

    /**
     * \brief   Initialize the waypoint handler
     *
     * \param   position_estimation The reference to the position estimation
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
     * \param   radius              The radius of the waypoint
     * \param   loiter_time         The time to loiter at the waypoint
     * \param   dubin               The dubin structure for the waypoint
     */
    Waypoint(   const Position_estimation* position_estimation,
                uint8_t frame,
                uint16_t command,
                uint8_t autocontinue,
                float param1,
                float param2,
                float param3,
                float param4,
                float x,
                float y,
                float z,
                float radius,
                float loiter_time,
                dubin_t dubin);

    /**
     * \brief   Calculates the information required for waypoint local structure
     *
     * \param   dubin_state             The pointer to the Dubin state
     */
    void calculate_waypoint_local_structure(dubin_state_t* dubin_state);

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
     * \brief   Gets the waypoint in local coordinates
     *
     * \return  Local waypoint position
     */
    local_position_t local_pos() const;

    /**
     * \brief   Sets the waypoint in local coordinates
     */
    void set_local_pos(local_position_t local_pos);

    /**
     * \brief   Gets the radius
     *
     * \return  radius
     */
    float radius() const;

    /**
     * \brief   Sets the radius
     *
     * \param   radius
     */
    void set_radius(float radius);

    /**
     * \brief   Gets the loiter time of the waypoint
     *
     * \return  Loiter time
     */
    float loiter_time() const;

    /**
     * \brief   Sets the loiter time
     *
     * \param   loiter time
     */
    void set_loiter_time(float loiter_time);

    /**
     * \brief   Gets a reference to the dubin structure
     *
     * \return  Dubin structure for the waypoint
     */
    dubin_t& dubin();

protected:
    uint8_t frame_;                                             ///< The reference frame of the waypoint
    uint16_t command_;                                          ///< The MAV_CMD_NAV id of the waypoint
    uint8_t autocontinue_;                                      ///< Flag to tell whether the vehicle should auto continue to the next waypoint once it reaches the current waypoint
    float param1_;                                              ///< Parameter depending on the MAV_CMD_NAV id
    float param2_;                                              ///< Parameter depending on the MAV_CMD_NAV id
    float param3_;                                              ///< Parameter depending on the MAV_CMD_NAV id
    float param4_;                                              ///< Parameter depending on the MAV_CMD_NAV id
    global_position_t wpt_position_;                            ///< The global position of the waypoint
    float radius_;                                              ///< The radius to turn around the waypoint, positive value for clockwise orbit, negative value for counter-clockwise orbit
    float loiter_time_;                                         ///< The loiter time at the waypoint
    dubin_t dubin_;                                             ///< The Dubin structure

    const Position_estimation* position_estimation_;            ///< The position estimation

    /**
     * \brief   Determines the global position of the waypoint based on the frame
     * and x, y, z, coordinates
     *
     * \param frame     The MAV_FRAME of the x, y, z coordinates
     * \param x         The x coordinate
     * \param y         The y coordinate
     * \param z         The z coordinate
     * \param heading   The heading of the waypoint in radians
     * \param origin    The origin of the local frame (if needed)
     *
     * \return  Global frame position
     */
    static global_position_t get_global_position(uint8_t frame, double x, double y, double z, float heading, global_position_t origin);

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
