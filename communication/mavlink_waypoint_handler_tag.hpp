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
 * \file mavlink_waypoint_handler_tag.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink tag waypoint handler
 *
 ******************************************************************************/


#ifndef MAVLINK_WAYPOINT_HANDLER_TAG__
#define MAVLINK_WAYPOINT_HANDLER_TAG__

#include "communication/mavlink_waypoint_handler.hpp"
#include "communication/mavlink_communication.hpp"
#include "communication/mavlink_stream.hpp"
#include "sensing/position_estimation.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "communication/state.hpp"
#include "sensing/qfilter.hpp"
#include "control/manual_control.hpp"
#include "control/navigation.hpp"
#include "sensing/offboard_tag_search.hpp"

#define MAX_WAYPOINTS 10        ///< The maximal size of the waypoint list
#define ALLOWABLE_HORIZONTAL_TAG_OFFSET_SQR 1.0        ///< The square distance from the drone to the center of the tag that is acceptable
/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */


class Mavlink_waypoint_handler_tag : public Mavlink_waypoint_handler
{
public:


    /**
     * \brief   Initialize the waypoint handler
     *
     * \param   position_estimation     The pointer to the position estimator structure
     * \param   navigation              The pointer to the navigation structure
     * \param   ahrs                    The pointer to the attitude estimation structure
     * \param   state                   The pointer to the state structure
     * \param   manual_control          The pointer to the manual control structure
     * \param   mavlink_communication   The pointer to the MAVLink communication structure
     * \param   mavlink_stream          The pointer to the MAVLink stream structure
     *
     * \return  True if the init succeed, false otherwise
     */
    Mavlink_waypoint_handler_tag(Position_estimation& position_estimation,
                           Navigation& navigation,
                           const ahrs_t& ahrs,
                           State& state,
                           const Manual_control& manual_control,
                           Mavlink_message_handler& message_handler,
                           const Mavlink_stream& mavlink_stream,
                           Offboard_Tag_Search& offboard_tag_search,
                           Mavlink_communication* raspi_mavlink_communication);


    

    static mav_result_t set_auto_landing(Mavlink_waypoint_handler_tag* waypoint_handler, mavlink_command_long_t* packet);

protected:
    Offboard_Tag_Search& offboard_tag_search_;
    Mavlink_communication* raspi_mavlink_communication_;

    void auto_land_on_tag_handler();
    virtual void state_machine();
};










#endif // MAVLINK_WAYPOINT_HANDLER_TAG__