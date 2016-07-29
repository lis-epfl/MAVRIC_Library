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
 * \file pwm_stm32.hpp
 *
 * \author MAV'RIC Team
 * \author Heitz Gregoire
 *
 * \brief   This file is the driver for pwm servos
 *
 ******************************************************************************/


#ifndef PWM_SERVOS_STM32_HPP_
#define PWM_SERVOS_STM32_HPP_

#include <cstdbool>
#include <cstdint>

#include "hal/common/pwm.hpp"

#include "hal/stm32/gpio_stm32.hpp"

extern "C"
{
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
}


class Pwm_stm32: public Pwm
{
public:

    /**
     * \brief   enum of timer channel
     */
    typedef enum
    {
        PWM_STM32_CHANNEL_1,
        PWM_STM32_CHANNEL_2,
        PWM_STM32_CHANNEL_3,
        PWM_STM32_CHANNEL_4
    } pwm_stm32_channel_t;

    /**
     * \brief   STM32_PWM configuration structure
     */
    typedef struct
    {
        gpio_stm32_conf_t       gpio_config;            ///< specify which gpio is used
        uint32_t                timer_config;           ///< specify which timer is used
        rcc_periph_clken        rcc_timer_config;       ///< rcc reference to the timer
        pwm_stm32_channel_t     channel_config;         ///< specify which channel is used
        uint32_t                prescaler_config;       ///< specify the clk divider
        uint32_t                period_config;          ///< specify the period of the PWM
        uint32_t                duty_cycle_config;      ///< specify the pulse length in microseconds
    } config_t;


    /**
     * \brief Constructor
     *
     * \param Servo number (between 0 and 7)
     */
    Pwm_stm32(config_t pwm_config);


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

    config_t pwm_config_;

    uint8_t channel_id_;    ///< PWM channel number
    uint32_t timer_;        ///< TIMER used
    uint32_t prescaler_;    ///< clk divider
    uint32_t period_;       ///< period
    uint32_t duty_cycle_;   ///< pulse length in us
};

#endif /* PWM_STM32_HPP_ */
