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
 * \file servo.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Driver for servomotors using PWM
 *
 ******************************************************************************/


#ifndef SERVO_HPP_
#define SERVO_HPP_

#include <stdint.h>
#include <stdbool.h>

#include "hal/common/pwm.hpp"


/**
 * \brief   Configuration structure for servo
 */
typedef struct
{
    float    trim;                  ///< Trim value (between -1 and 1)
    float    min;                   ///< Minimum value (between -1 and 1)
    float    max;                   ///< Max value (between -1 and 1)
    float    failsafe;              ///< Failsafe position of the servo (between -1 and 1)
    uint32_t repeat_freq;           ///< Update frequency of the servo (in Hz)
    uint16_t pulse_center_us;       ///< Pulse width in microseconds for neutral servo position
    uint16_t pulse_magnitude_us;    ///< Amplitude of variation of the pulse width
} servo_conf_t;


/**
 * \brief   Default configuration for standard servos
 *
 * \return  Conf structure
 */
static inline servo_conf_t servo_default_config_standard();


/**
 * \brief   Default configuration for ESCs (motor controller)
 *
 * \return  Conf structure
 */
static inline servo_conf_t servo_default_config_esc();


/**
 * \brief Driver for servomotors using PWM.
 */
class Servo
{
public:
    /**
     * \brief   Constructor
     *
     * \param   config      Configuration
     */
    Servo(Pwm& pwm, const servo_conf_t config = servo_default_config_standard());


    /**
     * \brief   Returns the current servos value
     *
     * \return  Value
     */
    float read(void) const;


    /**
     * \brief   Sets the servos to a given value
     *
     * \param   value           The servo value to be set
     * \param   to_hardware     Indicates if hardware peripheral should be updated (false by default)
     *
     * \return  Success
     */
    bool write(float value, bool to_hardware = true);


    /**
     * \brief   Sets the servos to failsafe value
     *
     * \param   to_hardware     Indicates if hardware peripheral should be updated (false by default)
     *
     * \return  Success
     */
    bool failsafe(bool to_hardware = true);


    /**
     * \brief   Set PWM line according to the servo value
     *
     * \return  Success
     */
    bool write_to_hardware(void);


    /**
     * \brief   Perform ESC calibration
     *
     * \details DO NOT USE IN FLIGHT !
     *          Set output to max for 2 seconds, then to failsafe value
     */
    void calibrate_esc(void);


private:
    Pwm&            pwm_;       ///< Reference to pwm
    servo_conf_t    config_;    ///< Configuration
    float           value_;     ///< Normalized value of the servo (between -1 and 1)
};


/**
 * \brief   Default configuration for standard servos
 *
 * \return  Conf structure
 */
static inline servo_conf_t servo_default_config_standard()
{
    servo_conf_t conf       = {};

    conf.trim               = 0.0f;
    conf.min                = -1.0f;
    conf.max                = 1.0f;
    conf.failsafe           = 0.0f;
    conf.repeat_freq        = 50;
    conf.pulse_center_us    = 1500;
    conf.pulse_magnitude_us = 500;

    return conf;
};


/**
 * \brief   Default configuration for ESCs (motor controller)
 *
 * \return  Conf structure
 */
static inline servo_conf_t servo_default_config_esc()
{
    servo_conf_t conf       = {};

    conf.trim               = 0.0f;
    conf.min                = -0.9f;
    conf.max                = 1.0f;
    conf.failsafe           = -1.1f;
    conf.repeat_freq        = 200;
    conf.pulse_center_us    = 1500;
    conf.pulse_magnitude_us = 500;

    return conf;
};


#endif /* SERVO_HPP_ */