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
 * \file mission_handler_land_on_tag.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the land on tag state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_LAND_ON_TAG_HPP__
#define MISSION_HANDLER_LAND_ON_TAG_HPP__

#include "communication/mavlink_communication.hpp"
#include "communication/state.hpp"
#include "mission/mission_handler.hpp"
#include "sensing/offboard_tag_search.hpp"

/*
 * The handler class takes in a template parameter that allows control inputs.
 */
template <class T1, class T2, class T3>
class Mission_handler_land_on_tag : public Mission_handler
{
public:
    /**
     * \brief   The auto-landing enum
     */
    typedef enum
    {
        FLY_TO_LANDING_LOCATION,                            ///< First auto landing behavior, fly to the landing location
        DESCENT_TO_SMALL_ALTITUDE,                          ///< Second auto landing behavior, descend to small altitude over tag
        DESCENT_TO_GND                                      ///< Third auto landing behavior, descend to ground
    } auto_landing_behavior_t;

    /**
     * \brief The landing mission handler configuration structure
     */
    struct conf_t
    {
        float LPF_gain;                                     ///< The value of the low-pass filter gain
        float desc_to_ground_altitude;                      ///< The altitude to switch to the descent to ground state
        float desc_to_ground_range;                         ///< The range in meters that allows switching from the descent to ground
        float min_waypoint_acceptance_radius;               ///< The minimum acceptable radius to start the landing procedure
    };

    /**
     * \brief   Initialize the land on tag mission planner handler
     *
     * \param   fly_to_landing_location_controller  The reference to the controller used during the fly to landing location phase
     * \param   desc_to_small_alt_controller        The reference to the controller used during the descent to small altitudes phase
     * \param   desc_to_ground_controller           The reference to the controller used during the descent to ground phase
     * \param   ahrs                                The reference to the ahrs structure
     * \param   ins                                 The reference to the ins
     * \param   state                               The reference to the state structure
     * \param   offboard_tag_search                 The reference to the offboard tag search module
     * \param   mission_planner                     The reference to the mission planner module
     * \param   message_handler                     The reference to the mavlink message handler module
     * \param   config                              The landing mission handler config structure
     */
     Mission_handler_land_on_tag(   T1& fly_to_landing_location_controller,
                                    T2& desc_to_small_alt_controller,
                                    T3& desc_to_ground_controller,
                                    const ahrs_t& ahrs,
                                    const INS& ins,
                                    State& state,
                                    Offboard_Tag_Search& offboard_tag_search,
                                    Mission_planner& mission_planner,
                                    Mavlink_message_handler& message_handler,
                                    conf_t config = default_config());

    /**
     * \brief   Checks if the waypoint is a land on tag waypoint
     *  
     * \details     Checks if the inputted waypoint is a:
     *                  MAV_CMD_NAV_LAND_ON_TAG
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Can handle
     */
    virtual bool can_handle(const Waypoint& wpt) const;

    /**
     * \brief   Sets up this handler class for a first time initialization
     *  
     * \details     Records the waypoint reference and sets the mav mode
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Success
     */
    virtual bool setup(const Waypoint& wpt);

    /**
     * \brief   Handles the mission every iteration
     *  
     * \details     Sets the goal location and determines the status code. The
     *              code is MISSION_IN_PROGRESS for landing in progress, MISSION_FINISHED for landing finished
     *              and autocontinue on, MISSION_FAILED for control command rejected.
     *
     * \return  Status code
     */
    virtual Mission_handler::update_status_t update();

    /**
     * \brief   Returns that the mission state is in POSTMISSION
     *
     * \return  Mission handler's mission state
     */
    virtual Mission_planner::internal_state_t handler_mission_state() const;

    /**
     * \brief   default configuration for the landing handler
     *
     * \return default config
     */
    static inline conf_t default_config();

protected:
    Waypoint waypoint_;                                         ///< The waypoint that we are landing under
    bool is_landed_;                                            ///< Boolean flag stating that we have finished the landing procedure
    auto_landing_behavior_t auto_landing_behavior_;             ///< The auto landing behavior
    float alt_lpf_;                                             ///< The low-pass filtered altitude for auto-landing
    float LPF_gain_;                                            ///< The low-pass filter gain
    float desc_to_ground_altitude_;                             ///< The altitude to switch to the descent to ground state
    float desc_to_ground_range_;                                ///< The range in meters that allows switching from the descent to ground
    float min_waypoint_acceptance_radius_;                      ///< The minimum acceptable radius to start the landing procedure
    float tag_search_altitude_;                                 ///< The altitude that the drone should search for the tag at
    uint32_t tag_search_start_time_;                            ///< The start time that the offboard camera has been searching for the tag, causes a timeout if too long

    T1& fly_to_landing_location_controller_;                    ///< The reference to the controller used during the fly to landing location phase
    T2& desc_to_small_alt_controller_;                          ///< The reference to the controller used during the descent to small altitudes phase
    T3& desc_to_ground_controller_;                             ///< The reference to the controller used during the descent to ground phase
    const ahrs_t& ahrs_;                                        ///< The reference to the ahrs structure
    const INS& ins_;                                            ///< The reference to the ins interface
    State& state_;                                              ///< The reference to the state structure
    Offboard_Tag_Search& offboard_tag_search_;
    Mission_planner& mission_planner_;
    
    /**
     * \brief   Function to set the controller specific command for the fly to
     *          landing location
     *
     * \return  Controller accepted input
     */
    virtual bool set_fly_to_landing_location_control_command();

    /**
     * \brief   Function to set the controller specific command for the descent to
     *          small altitude state
     *
     * \return  Controller accepted input
     */
    virtual bool set_desc_to_small_alt_control_command();

    /**
     * \brief   Function to set the controller specific command for the descent to
     *          ground state
     *
     * \return  Controller accepted input
     */
    virtual bool set_desc_to_ground_control_command();

    /**
     * \brief   Sets the autolanding on tag handler
     *
     * \param   land_on_tag_handler The pointer to the land on tag handler
     * \param   packet              The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t set_auto_landing_tag(Mission_handler_land_on_tag* land_on_tag_handler, const mavlink_command_long_t* packet);
};

template <class T1, class T2, class T3>
typename Mission_handler_land_on_tag<T1, T2, T3>::conf_t Mission_handler_land_on_tag<T1, T2, T3>::default_config()
{
    conf_t conf                                      = {};

    conf.LPF_gain                                    = 0.9f;
    conf.desc_to_ground_altitude                     = -1.5f;
    conf.desc_to_ground_range                        = 0.5f;
    conf.min_waypoint_acceptance_radius              = 3.0f;

    return conf;
};

#include "mission/mission_handler_land_on_tag.hxx"

#endif // MISSION_HANDLER_LAND_ON_TAG_HPP__
