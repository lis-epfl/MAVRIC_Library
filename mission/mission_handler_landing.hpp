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
 * \file mission_handler_landing.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the landing state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_LANDING__
#define MISSION_HANDLER_LANDING__

#include "status/state.hpp"
#include "mission/mission_handler.hpp"

/*
 * The handler class takes in a template parameter that allows control inputs.
 */
template <class T1, class T2>
class Mission_handler_landing : public Mission_handler
{
public:
    /**
     * \brief   The auto-landing enum
     */
    typedef enum
    {
        DESCENT_TO_SMALL_ALTITUDE,                          ///< First auto landing behavior
        DESCENT_TO_GND                                      ///< Second auto landing behavior, comes after DESCENT_TO_SMAL_ALTITUDE
    } auto_landing_behavior_t;

    /**
     * \brief The landing mission handler configuration structure
     */
    struct conf_t
    {
        float LPF_gain;                                     ///< The value of the low-pass filter gain
        float desc_to_ground_altitude;                      ///< The altitude to switch to the descent to ground state
        float desc_to_ground_range;                         ///< The range in meters that allows switching from the descent to ground
    };

    /**
     * \brief   Initialize the landing mission planner handler
     *
     * \param   desc_to_small_alt_controller    The reference to the controller used during the descent to small altitudes phase
     * \param   desc_to_ground_controller       The reference to the controller used during the descent to ground phase
     * \param   ins                             The reference to the ins
     * \param   state                           The reference to the state structure
     * \param   config                          The landing mission handler config structure
     */
     Mission_handler_landing(   T1& desc_to_small_alt_controller,
                                T2& desc_to_ground_controller,
                                const INS& ins,
                                State& state,
                                conf_t config = default_config());

    /**
     * \brief   Checks if the waypoint is a landing waypoint
     *  
     * \details     Checks if the inputted waypoint is a:
     *                  MAV_CMD_NAV_LAND
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
    
    T1& desc_to_small_alt_controller_;                          ///< The reference to the controller used during the descent to small altitudes phase
    T2& desc_to_ground_controller_;                             ///< The reference to the controller used during the descent to ground phase
    const INS& ins_;                                            ///< The reference to the ins interface
    State& state_;                                              ///< The reference to the state structure

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
};

template <class T1, class T2>
typename Mission_handler_landing<T1, T2>::conf_t Mission_handler_landing<T1, T2>::default_config()
{
    conf_t conf                                      = {};

    conf.LPF_gain                                    = 0.9f;
    conf.desc_to_ground_altitude                     = -5.0f;
    conf.desc_to_ground_range                        = 0.5f;

    return conf;
};

#include "mission/mission_handler_landing.hxx"

#endif // MISSION_HANDLER_LANDING__
