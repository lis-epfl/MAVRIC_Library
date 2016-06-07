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
 * \file mav_mode.h
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Place where the mav modes and flags are defined
 *
 ******************************************************************************/


#ifndef MAV_MODE_H
#define MAV_MODE_H

#include "communication/mavlink_stream.hpp"

extern "C"
{
#include <stdint.h>
#include <stdbool.h>
#include "util/print_util.h"
}

typedef enum MAV_STATE mav_state_t;

class Mav_mode
{
public:
    typedef uint8_t mode_bits_t;
    typedef uint8_t bitmask_t;
    typedef uint32_t custom_mode_t;


    /*
     * \brief enum for defining custom mode
     */
    typedef enum
    {
        CUSTOM_BASE_MODE = 0,
        CUST_CRITICAL_CLIMB_TO_SAFE_ALT = 1,        ///< First critical behavior
        CUST_CRITICAL_FLY_TO_HOME_WP = 2,           ///< Second critical behavior, comes after CLIMB_TO_SAFE_ALT
        CUST_CRITICAL_LAND = 4,                     ///< Third critical behavior, comes after FLY_TO_HOME_WP

        CUST_DESCENT_TO_SMALL_ALTITUDE = 8,         ///< First auto landing behavior
        CUST_DESCENT_TO_GND = 16,                   ///< Second auto landing behavior, comes after DESCENT_TO_SMAL_ALTITUDE

        CUST_COLLISION_AVOIDANCE = 32,              ///< Collision avoidance

        CUST_BATTERY_LOW = 64,                      ///< Battery low flag
        CUST_FENCE_1 = 128,                         ///< Fence 1 violation flag
        CUST_FENCE_2 = 256,                         ///< Fence 2 violation flag
        CUST_HEARTBEAT_LOST = 512,                  ///< Heartbeat loss flag
        CUST_REMOTE_LOST = 1024,                    ///< Remote lost flag
        CUST_GPS_BAD = 2048                         ///< GPS loss flag
    } custom_mode_list_t;


    /* Control modes (predefined mode taking into account manual, stabilize, guided, and auto flag) */
    enum ctrl_mode_t
    {
        RATE =          MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        ATTITUDE =      MAV_MODE_FLAG_MANUAL_INPUT_ENABLED + MAV_MODE_FLAG_STABILIZE_ENABLED,
        VELOCITY =      MAV_MODE_FLAG_MANUAL_INPUT_ENABLED + MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_GUIDED_ENABLED,
        POSITION_HOLD = MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_GUIDED_ENABLED,
        GPS_NAV =       MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_GUIDED_ENABLED + MAV_MODE_FLAG_AUTO_ENABLED
    };

    /* describes which flags are used for ctrl_mode */
    static const bitmask_t CTRL_MODE_BITFIELD = 0b01011100;

    Mav_mode() : bits_(0){};

    Mav_mode(const mode_bits_t& bits) : bits_(bits){};

    Mav_mode(const Mav_mode& mav_mode) : bits_(mav_mode.bits_){};

    /**
     * \brief     returns the mode as mode_bits (uint8_t)
     *
     * \return    bits of mode
     */
    inline mode_bits_t bits() const {return bits_;};

    /**
     * \brief   returns a pointer to bits_ where each bit represents a flag
     *
     * \details this function is temporary as long as data logging is using pointers
     *
     * \return  pointer to bits
     */
    inline const uint8_t* bits_ptr() const {return static_cast<const uint8_t*>(&bits_);};


    /**
     * \brief     returns whether armed (MAV_MODE_FLAG_SAFETY_ARMED set)
     *
     * \return    MAV_MODE_FLAG_SAFETY_ARMED
     */
    inline bool is_armed() const {return is_flag_set(MAV_MODE_FLAG_SAFETY_ARMED);};

    /**
     * \brief     returns whether in manual mode (MAV_MODE_FLAG_MANUAL_INPUT_ENABLED set)
     *
     * \return    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
     */
    inline bool is_manual() const {return is_flag_set(MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);};

    /**
     * \brief     returns whether in HIL mode (MAV_MODE_FLAG_HIL_ENABLED set)
     *
     * \return    MAV_MODE_FLAG_HIL_ENABLED
     */
    inline bool is_hil() const {return is_flag_set(MAV_MODE_FLAG_HIL_ENABLED);};

    /**
     * \brief     returns whether in stabilize mode (MAV_MODE_FLAG_STABILIZE_ENABLED set)
     *
     * \return    MAV_MODE_FLAG_STABILIZE_ENABLED
     */
    inline bool is_stabilize() const {return is_flag_set(MAV_MODE_FLAG_STABILIZE_ENABLED);};

    /**
     * \brief     returns whether in guided mode (MAV_MODE_FLAG_GUIDED_ENABLED set)
     *
     * \return    MAV_MODE_FLAG_GUIDED_ENABLED
     */
    inline bool is_guided() const {return is_flag_set(MAV_MODE_FLAG_GUIDED_ENABLED);};

    /**
     * \brief     returns whether in auto mode (MAV_MODE_FLAG_AUTO_ENABLED set)
     *
     * \return    MAV_MODE_FLAG_AUTO_ENABLED
     */
    inline bool is_auto() const {return is_flag_set(MAV_MODE_FLAG_AUTO_ENABLED);};

    /**
     * \brief     returns whether in test mode (MAV_MODE_FLAG_TEST_ENABLED set)
     *
     * \return    MAV_MODE_FLAG_TEST_ENABLED
     */
    inline bool is_test() const {return is_flag_set(MAV_MODE_FLAG_TEST_ENABLED);};

    /**
     * \brief     returns whether custom mode enabled (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED set)
     *
     * \return    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
     */
    inline bool is_custom() const {return is_flag_set(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);};

    /**
     * \brief     returns the ctrl_mode (defined by manual, stabilize, guided and auto flag)
     *
     * \return    ctrl_mode
     */
    inline ctrl_mode_t ctrl_mode() const {return (ctrl_mode_t)(bits_ & CTRL_MODE_BITFIELD);};

    /**
     * \brief   set armed flag
     *
     * \param   value      true for arming, false for disarming
     *
     */
    inline void set_armed_flag(bool value) { set_flag(MAV_MODE_FLAG_SAFETY_ARMED, value);};

    /**
     * \brief   set manual flag
     *
     * \param   value      true for passing to manual, false for not manual
     *
     */
    inline void set_manual_flag(bool value) { set_flag(MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, value);};

    /**
     * \brief   set HIL flag (Hardware in the loop)
     *
     * \param   value          true for passing to HIL, false for not HIL
     *
     */
    inline void set_hil_flag(bool value) { set_flag(MAV_MODE_FLAG_HIL_ENABLED, value);};

    /**
     * \brief   set stabilize flag
     *
     * \param   value          true for passing to stabilize, false for not stabilize
     *
     */
    inline void set_stabilize_flag(bool value) { set_flag(MAV_MODE_FLAG_STABILIZE_ENABLED, value);};

    /**
     * \brief   set guided flag
     *
     * \param   value          true for passing to guided, false for not guided
     *
     */
    inline void set_guided_flag(bool value) { set_flag(MAV_MODE_FLAG_GUIDED_ENABLED, value);};

    /**
     * \brief   set auto flag
     *
     * \param   value          true for passing to auto, false for not auto
     *
     */
    inline void set_auto_flag(bool value) { set_flag(MAV_MODE_FLAG_AUTO_ENABLED, value);};

    /**
     * \brief   set test flag
     *
     * \param   value          true for passing to test, false for not test
     *
     */
    inline void set_test_flag(bool value) { set_flag(MAV_MODE_FLAG_TEST_ENABLED, value);};

    /**
     * \brief   set custom_mode flag
     *
     * \param   value          true for passing to test, false for not test
     *
     */
    inline void set_custom_flag(bool value)  { set_flag(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, value);};

    /**
     * \brief   set ctrl_mode (RATE, ATTITUDE, VELOCITY, POSITION_HOLD, GPS_NAV);
     *
     * \details affects only manual, stabilize, guided and auto flag
     */
    inline void set_ctrl_mode(ctrl_mode_t ctrl_mode) {bits_ = (bits_ & ~CTRL_MODE_BITFIELD) + (ctrl_mode & CTRL_MODE_BITFIELD);};

    /**
     * \brief   operator overload for ==  (calls this.equal(mav_mode) )
     * 
     * \return  true if equal, else false
     */
    bool operator == (const Mav_mode& mav_mode) const {return equal(mav_mode);};

private:
    mode_bits_t bits_;

    /**
     * \brief     check a single flag of the mode
     *
     * \param     flag      flag to be checked
     *
     * \return    value     true if flag is set, false otherwise
     */
    inline bool is_flag_set(MAV_MODE_FLAG flag) const {return ((bits_ & flag) == flag);};

    /**
     * \brief     set a single flag of the mode
     *
     * \param     flag      flag to be set
     * \param     value     value the flag is set to
     */
    inline void set_flag(MAV_MODE_FLAG flag, bool value)  { bits_ = value ? bits_ | flag : bits_ & ~flag;};

    /**
     * \brief   compare to another Mav_mode
     *
     * \return  true if equal, else false
     */
     inline bool equal(Mav_mode mav_mode) const {return bits_ == mav_mode.bits_;};

    /**
     * \brief   set mode to corresponding bits
     *
     * \param   new_mode      new mode
     *
     */
    inline void set_mode(mode_bits_t bits) {bits_ = bits;};

};


/**
 * \brief Enum defining the arming/disarming events
 */
typedef enum
{
    ARM_ACTION_DISARMING    = -1,   ///< The next action is the disarming of the motors
    ARM_ACTION_NONE         = 0,    ///< The next action is nothing
    ARM_ACTION_ARMING       = 1,    ///< The next action is arming the motors
} arm_action_t;


#endif //MAV_MODE_H
