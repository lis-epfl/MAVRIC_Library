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
 * \file joystick.hpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief This file is to decode the set manual command message from MAVLink
 *
 ******************************************************************************/


#ifndef JOYSTICK_HPP_
#define JOYSTICK_HPP_

#include "status/state.hpp"
#include "control/stabilisation.hpp"
#include "control/control_command.hpp"
#include "control/controller.hpp"



#define MAX_JOYSTICK_RANGE 0.8  ///< Scale down the joystick channel amplitude, as done in remote


class Joystick
{
public:

    /**
     * \brief button enumeration
     */
    typedef enum
    {
        BUTTON_UNPRESSED = 0,
        BUTTON_PRESSED = 1,
    } button_pressed_t;

    enum class throttle_mode_t
    {
        ZERO_CENTER = 0,    //< throttle stick in center is zero throttle
        ZERO_DOWN           //< throttle stick all the way down is zero throttle
    };



    /**
     * \brief   The union structure for the bit mask of the joystick buttons
     */
    typedef union
    {
        uint16_t button_mask;
        // unamed bitfield structure, use to access directly the flags
        struct
        {
            button_pressed_t        button_16   : 1;
            button_pressed_t        button_15   : 1;
            button_pressed_t        button_14   : 1;
            button_pressed_t        button_13   : 1;
            button_pressed_t        button_12   : 1;
            button_pressed_t        button_11   : 1;
            button_pressed_t        button_10   : 1;
            button_pressed_t        button_9    : 1;
            button_pressed_t        button_8    : 1;
            button_pressed_t        button_7    : 1;
            button_pressed_t        button_6    : 1;
            button_pressed_t        button_5    : 1;
            button_pressed_t        button_4    : 1;
            button_pressed_t        button_3    : 1;
            button_pressed_t        button_2    : 1;
            button_pressed_t        button_1    : 1;
        };
        // identical bitfield, but named (useful for initialisation)
        struct
        {
            button_pressed_t        button_16   : 1;
            button_pressed_t        button_15   : 1;
            button_pressed_t        button_14   : 1;
            button_pressed_t        button_13   : 1;
            button_pressed_t        button_12   : 1;
            button_pressed_t        button_11   : 1;
            button_pressed_t        button_10   : 1;
            button_pressed_t        button_9    : 1;
            button_pressed_t        button_8    : 1;
            button_pressed_t        button_7    : 1;
            button_pressed_t        button_6    : 1;
            button_pressed_t        button_5    : 1;
            button_pressed_t        button_4    : 1;
            button_pressed_t        button_3    : 1;
            button_pressed_t        button_2    : 1;
            button_pressed_t        button_1    : 1;
        } button;
    } button_t;

    /**
     * \brief  Joystick Channels
     */
    struct channels_t
    {
        float x;    // Longitudinal (pitch)
        float y;    // Lateral      (roll)
        float z;    // Vertical     (thrust)
        float r;    // Rotation     (yaw)
    };


    /**
     * \brief  Configuration for joystick
     */
    struct conf_t
    {
        throttle_mode_t    throttle_mode;  ///< indicates whether zero throttle is stick in center or stick down
        channels_t         scale_attitude; ///< scales applied to channels in attitude mode
        channels_t         scale_velocity; ///< scales applied to channels in velocity mode
    };


    /**
     * \brief   Constructor
     */
    Joystick(conf_t conf = default_config());

    /**
     * \brief   Returns the throttle value from the joystick
     *
     * \return  The value of the throttle
     */
    float throttle() const;


    /**
     * \brief   Returns the roll value from the joystick
     *
     * \return  The value of the roll
     */
    float roll() const;


    /**
     * \brief   Returns the pitch value from the joystick
     *
     * \return  The value of the pitch
     */
    float pitch() const;


    /**
     * \brief   Returns the yaw value from the joystick
     *
     * \return  The value of the yaw
     */
    float yaw() const;


    /**
     * \brief   Returns the current desired mode value from the joystick
     *
     * \return  The value of the current desired mode
     */
    Mav_mode get_mode(const Mav_mode current_mode) ;

    /**
     * \brief               Do operations when buttons are pressed
     *
     * \param   buttons     The bit mask of the buttons
     */
    void button_update(uint16_t buttons);


    /**
     * \brief   Default config for joystick
     */
    static inline conf_t default_config();


    button_t                    buttons_;            ///< The bit mask of the button pressed
    channels_t                  channels_;           ///< Channels of the joystick
    Mav_mode                    mav_mode_desired_;   ///< The mav mode indicated by the remote

private:
    arm_action_t                arm_action_;
    throttle_mode_t             throttle_mode_;      ///< indicates whether zero throttle is stick in center or stick down
};


Joystick::conf_t Joystick::default_config()
{
    conf_t conf;
    conf.throttle_mode = throttle_mode_t::ZERO_DOWN;
    return conf;
}

#endif // JOYSTICK_HPP_
