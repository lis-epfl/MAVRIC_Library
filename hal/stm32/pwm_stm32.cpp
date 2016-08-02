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
 * \file pwm_stm32.cpp
 *
 * \author MAV'RIC Team
 * \author Heitz Gregoire
 *
 * \brief This file is the driver for pwm servos
 *
 ******************************************************************************/

#include "hal/stm32/pwm_stm32.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
#include <math.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Pwm_stm32::Pwm_stm32(config_t config)
{
    config_ = config;

    timer_         = config.timer;
    prescaler_     = config.prescaler;
    period_        = config.period;
    pulse_us_      = config.pulse_us;
    channel_id_    = config.channel;
}

bool Pwm_stm32::init(void)
{
    bool success = true;

    /* Enable peripheral port & TIM clock. */
    //rcc_periph_clock_enable(RCC_GPIOx);
    rcc_periph_clock_enable(config_.rcc_timer);

    gpio_mode_setup(config_.gpio_config.port, GPIO_MODE_AF, GPIO_PUPD_NONE, config_.gpio_config.pin);
    gpio_set_af(config_.gpio_config.port, config_.gpio_config.alt_fct, config_.gpio_config.pin);
    gpio_set_output_options(config_.gpio_config.port, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, config_.gpio_config.pin);

    //WARNING Common to all channels of that TIMER
    //select prescaler
    TIM_PSC(config_.timer) = prescaler_;
    //select the output period
    TIM_ARR(config_.timer) = period_;
    //enable the autoreload
    TIM_CR1(config_.timer) |= TIM_CR1_ARPE;
    //select counting mode (edge-aligned)
    TIM_CR1(config_.timer) |= TIM_CR1_CMS_EDGE;
    //counting up
    TIM_CR1(config_.timer) |= TIM_CR1_DIR_UP;
    //enable counter
    TIM_CR1(config_.timer) |= TIM_CR1_CEN;

    //CHANNEL SPECIFIC
    if (config_.channel == PWM_STM32_CHANNEL_1)
    {
        //Disable channel1
        TIM_CCER(config_.timer) &= (uint16_t)~TIM_CCER_CC1E;
        //Reset output compare
        TIM_CCMR1(config_.timer) &= (uint16_t)~TIM_CCMR1_OC1M_MASK;
        TIM_CCMR1(config_.timer) &= (uint16_t)~TIM_CCMR1_CC1S_MASK;

        //Select output mode
        TIM_CCMR1(config_.timer) |= TIM_CCMR1_CC1S_OUT;
        //select polarity low
        TIM_CCER(config_.timer) |= TIM_CCER_CC1NP;
        //select PWM mode 1
        TIM_CCMR1(config_.timer) |= TIM_CCMR1_OC1M_PWM1;

        //select duty cycle
        TIM_CCR1(config_.timer) = pulse_us_;

        //set the preload bit
        TIM_CCMR1(config_.timer) |= TIM_CCMR1_OC1PE;

        //enable capture/compare
        TIM_CCER(config_.timer) |= TIM_CCER_CC1E;
    }
    else if (config_.channel == PWM_STM32_CHANNEL_2)
    {
        //Disable channel2
        TIM_CCER(config_.timer) &= (uint16_t)~TIM_CCER_CC2E;
        //Reset output compare
        TIM_CCMR1(config_.timer) &= (uint16_t)~TIM_CCMR1_OC2M_MASK;
        TIM_CCMR1(config_.timer) &= (uint16_t)~TIM_CCMR1_CC2S_MASK;

        //Select output mode
        TIM_CCMR1(config_.timer) |= TIM_CCMR1_CC2S_OUT;
        //select polarity low
        TIM_CCER(config_.timer) |= TIM_CCER_CC2NP;
        //select PWM mode 1
        TIM_CCMR1(config_.timer) |= TIM_CCMR1_OC2M_PWM1;

        //select duty cycle
        TIM_CCR2(config_.timer) = pulse_us_;

        //set the preload bit
        TIM_CCMR1(config_.timer) |= TIM_CCMR1_OC2PE;

        //enable capture/compare
        TIM_CCER(config_.timer) |= TIM_CCER_CC2E;
    }
    else if (config_.channel == PWM_STM32_CHANNEL_3)
    {
        //Disable channel3
        TIM_CCER(config_.timer) &= (uint16_t)~TIM_CCER_CC3E;
        //Reset output compare
        TIM_CCMR2(config_.timer) &= (uint16_t)~TIM_CCMR2_OC3M_MASK;
        TIM_CCMR2(config_.timer) &= (uint16_t)~TIM_CCMR2_CC3S_MASK;

        //Select output mode
        TIM_CCMR2(config_.timer) |= TIM_CCMR2_CC3S_OUT;
        //select polarity low
        TIM_CCER(config_.timer) |= TIM_CCER_CC3NP;
        //select PWM mode 1
        TIM_CCMR2(config_.timer) |= TIM_CCMR2_OC3M_PWM1;

        //select duty cycle
        TIM_CCR3(config_.timer) = pulse_us_;

        //set the preload bit
        TIM_CCMR2(config_.timer) |= TIM_CCMR2_OC3PE;

        //enable capture/compare
        TIM_CCER(config_.timer) |= TIM_CCER_CC3E;
    }
    else if (config_.channel == PWM_STM32_CHANNEL_4)
    {
        //Disable channel4
        TIM_CCER(config_.timer) &= (uint16_t)~TIM_CCER_CC4E;
        //Reset output compare
        TIM_CCMR2(config_.timer) &= (uint16_t)~TIM_CCMR2_OC4M_MASK;
        TIM_CCMR2(config_.timer) &= (uint16_t)~TIM_CCMR2_CC4S_MASK;

        //Select output mode
        TIM_CCMR2(config_.timer) |= TIM_CCMR2_CC4S_OUT;
        //select polarity low
        TIM_CCER(config_.timer) |= (1 << 15); //TODO TIM_CCER_CC4NP does not exist in libopencm3 library
        //select PWM mode 1
        TIM_CCMR2(config_.timer) |= TIM_CCMR2_OC4M_PWM1;

        //select duty cycle
        TIM_CCR4(config_.timer) = pulse_us_;

        //set the preload bit
        TIM_CCMR2(config_.timer) |= TIM_CCMR2_OC4PE;

        //enable capture/compare
        TIM_CCER(config_.timer) |= TIM_CCER_CC4E;
    }

    return success;
}

bool Pwm_stm32::set_pulse_width_us(uint16_t pulse_us)
{
    pulse_us_ = pulse_us;
    write_channel();

    return true;
}


bool Pwm_stm32::set_period_us(uint16_t period_us)
{
    // WARNING this affect the period of the TIMER not only the specific channel
    period_ = period_us;
    write_channel();

    return true;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Pwm_stm32::write_channel(void)
{
    //select the output period
    if (TIM_ARR(timer_) > period_)
    {
        print_util_dbg_print("should keep slower period.\r\n");
    }
    else
    {
        TIM_ARR(timer_) = period_;
    }

    if(channel_id_ == PWM_STM32_CHANNEL_1)
    {
        //select duty cycle
        TIM_CCR1(timer_) = pulse_us_;
    }
    else if(channel_id_ == PWM_STM32_CHANNEL_2)
    {
        //select duty cycle
        TIM_CCR2(timer_) = pulse_us_;
    }
    else if(channel_id_ == PWM_STM32_CHANNEL_3)
    {
        //select duty cycle
        TIM_CCR3(timer_) = pulse_us_;
    }
    else if(channel_id_ == PWM_STM32_CHANNEL_4)
    {
        //select duty cycle
        TIM_CCR4(timer_) = pulse_us_;
    }
}
