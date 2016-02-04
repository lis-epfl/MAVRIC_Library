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
 * \file pwm_avr32.hpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 * \author Nicolas Dousse
 *
 * \brief   This file is the driver for pwm servos
 *
 ******************************************************************************/


#ifndef PWM_SERVOS_AVR32_H_
#define PWM_SERVOS_AVR32_H_

#include <stdbool.h>
#include <stdint.h>

#include "hal/common/pwm.hpp"


class Pwm_avr32: public Pwm
{
public:
    /**
     * \brief Constructor
     *
     * \param Servo number (between 0 and 7)
     */
    Pwm_avr32(uint8_t id);


    /**
     * \brief   Initialize the hardware line for servos
     *
     * \return  Success
     */
    bool init(void);


    /**
     * \brief   Set pulse width
     *
     * \param  pulse_us     Pulse length in us
     *
     * \return Success
     */
    bool set_pulse_width_us(uint16_t pulse_us);


    /**
     * \brief   Set pulse period
     *
     * \param   period_us   Pulse period in us
     *
     * \return  Success
     */
    bool set_period_us(uint16_t period_us);


private:
    /**
     * \brief   Output a PWM on one channel
     */
    void write_channel(void);


    uint8_t id_;            ///< PWM line number
    uint8_t channel_id_;    ///< PWM channel number

    // this is static because each channel controls 2 pwm lines and is thus
    // shared by two objects
    static uint32_t pulse_us_[8];
    static uint32_t period_us_[8];
};

#endif /* PWM_AVR32_H_ */
