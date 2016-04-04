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

Pwm_stm32::Pwm_stm32(pwm_conf_t pwm_config)
{
    pwm_config_ = pwm_config;

    timer_              = pwm_config.timer_config;
    prescaler_          = pwm_config.prescaler_config;
    period_             = pwm_config.period_config;
    duty_cyle_          = pwm_config.duty_cycle_config;
    channel_id_         = pwm_config.channel_config;
}

bool Pwm_stm32::init(void)
{
    bool success = true;
    
    /* Enable peripheral port & TIM clock. */
    //rcc_periph_clock_enable(RCC_GPIOx);
    rcc_periph_clock_enable(pwm_config_.rcc_timer_config);

    gpio_mode_setup(pwm_config_.gpio_config.port, GPIO_MODE_AF, GPIO_PUPD_NONE, pwm_config_.gpio_config.pin);
    gpio_set_af(pwm_config_.gpio_config.port, pwm_config_.gpio_config.alt_fct, pwm_config_.gpio_config.pin);
    gpio_set_output_options(pwm_config_.gpio_config.port, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, pwm_config_.gpio_config.port);
  
    //WARNING Common to all channels of that TIMER
    //select prescaler
    TIM_PSC(pwm_config_.timer_config) = prescaler_;
    //select the output period
    TIM_ARR(pwm_config_.timer_config) = period_;
    //enable the autoreload
    TIM_CR1(pwm_config_.timer_config) |= TIM_CR1_ARPE;
    //select counting mode (edge-aligned)
    TIM_CR1(pwm_config_.timer_config) |= TIM_CR1_CMS_EDGE;
    //counting up
    TIM_CR1(pwm_config_.timer_config) |= TIM_CR1_DIR_UP;
    //enable counter
    TIM_CR1(pwm_config_.timer_config) |= TIM_CR1_CEN;

    //CHANNEL SPECIFIC
    if (pwm_config_.channel_config == CHANNEL_1)
    {
        //Disable channel1
        TIM_CCER(pwm_config_.timer_config) &= (uint16_t)~TIM_CCER_CC1E; 
        //Reset output compare
        TIM_CCMR1(pwm_config_.timer_config) &= (uint16_t)~TIM_CCMR1_OC1M_MASK;
        TIM_CCMR1(pwm_config_.timer_config) &= (uint16_t)~TIM_CCMR1_CC1S_MASK;

        //Select output mode
        TIM_CCMR1(pwm_config_.timer_config) |= TIM_CCMR1_CC1S_OUT;
        //select polarity low
        TIM_CCER(pwm_config_.timer_config) |= TIM_CCER_CC1NP;
        //select PWM mode 1
        TIM_CCMR1(pwm_config_.timer_config) |= TIM_CCMR1_OC1M_PWM1;

        //select duty cycle
        TIM_CCR1(pwm_config_.timer_config) = duty_cyle_;

        //set the preload bit
        TIM_CCMR1(pwm_config_.timer_config) |= TIM_CCMR1_OC1PE;
        
        //enable capture/compare
        TIM_CCER(pwm_config_.timer_config) |= TIM_CCER_CC1E;
    }
    else if (pwm_config_.channel_config == CHANNEL_2)
    {
        //Disable channel2
        TIM_CCER(pwm_config_.timer_config) &= (uint16_t)~TIM_CCER_CC2E; 
        //Reset output compare
        TIM_CCMR1(pwm_config_.timer_config) &= (uint16_t)~TIM_CCMR1_OC2M_MASK;
        TIM_CCMR1(pwm_config_.timer_config) &= (uint16_t)~TIM_CCMR1_CC2S_MASK;

        //Select output mode
        TIM_CCMR1(pwm_config_.timer_config) |= TIM_CCMR1_CC2S_OUT;
        //select polarity low
        TIM_CCER(pwm_config_.timer_config) |= TIM_CCER_CC2NP;
        //select PWM mode 1
        TIM_CCMR1(pwm_config_.timer_config) |= TIM_CCMR1_OC2M_PWM1;

        //select duty cycle
        TIM_CCR2(pwm_config_.timer_config) = duty_cyle_;

        //set the preload bit
        TIM_CCMR1(pwm_config_.timer_config) |= TIM_CCMR1_OC2PE;
        
        //enable capture/compare
        TIM_CCER(pwm_config_.timer_config) |= TIM_CCER_CC2E;
    }
    else if (pwm_config_.channel_config == CHANNEL_3)
    {
        //Disable channel3
        TIM_CCER(pwm_config_.timer_config) &= (uint16_t)~TIM_CCER_CC3E; 
        //Reset output compare
        TIM_CCMR2(pwm_config_.timer_config) &= (uint16_t)~TIM_CCMR2_OC3M_MASK;
        TIM_CCMR2(pwm_config_.timer_config) &= (uint16_t)~TIM_CCMR2_CC3S_MASK;

        //Select output mode
        TIM_CCMR2(pwm_config_.timer_config) |= TIM_CCMR2_CC3S_OUT;
        //select polarity low
        TIM_CCER(pwm_config_.timer_config) |= TIM_CCER_CC3NP;
        //select PWM mode 1
        TIM_CCMR2(pwm_config_.timer_config) |= TIM_CCMR2_OC3M_PWM1;

        //select duty cycle
        TIM_CCR3(pwm_config_.timer_config) = duty_cyle_;

        //set the preload bit
        TIM_CCMR2(pwm_config_.timer_config) |= TIM_CCMR2_OC3PE;
        
        //enable capture/compare
        TIM_CCER(pwm_config_.timer_config) |= TIM_CCER_CC3E;
    }
    else if (pwm_config_.channel_config == CHANNEL_4)
    {
        //Disable channel4
        TIM_CCER(pwm_config_.timer_config) &= (uint16_t)~TIM_CCER_CC4E; 
        //Reset output compare
        TIM_CCMR2(pwm_config_.timer_config) &= (uint16_t)~TIM_CCMR2_OC4M_MASK;
        TIM_CCMR2(pwm_config_.timer_config) &= (uint16_t)~TIM_CCMR2_CC4S_MASK;

        //Select output mode
        TIM_CCMR2(pwm_config_.timer_config) |= TIM_CCMR2_CC4S_OUT;
        //select polarity low
        TIM_CCER(pwm_config_.timer_config) |= (1 << 15); //TODO TIM_CCER_CC4NP does not exist in libopencm3 library
        //select PWM mode 1
        TIM_CCMR2(pwm_config_.timer_config) |= TIM_CCMR2_OC4M_PWM1;

        //select duty cycle
        TIM_CCR4(pwm_config_.timer_config) = duty_cyle_;

        //set the preload bit
        TIM_CCMR2(pwm_config_.timer_config) |= TIM_CCMR2_OC4PE;
        
        //enable capture/compare
        TIM_CCER(pwm_config_.timer_config) |= TIM_CCER_CC4E;
    }

    return success;
}

bool Pwm_stm32::set_pulse_width_us(uint16_t pulse_us)
{
    duty_cyle_ = pulse_us;
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
    TIM_ARR(timer_) = period_;
    
    if(channel_id_ == CHANNEL_1)
    {
        //select duty cycle
        TIM_CCR1(timer_) = duty_cyle_;
    }
    else if(channel_id_ == CHANNEL_2)
    {
        //select duty cycle
        TIM_CCR2(timer_) = duty_cyle_;
    }
    else if(channel_id_ == CHANNEL_3)
    {
        //select duty cycle
        TIM_CCR3(timer_) = duty_cyle_;
    }
    else if(channel_id_ == CHANNEL_4)
    {
        //select duty cycle
        TIM_CCR4(timer_) = duty_cyle_;
    }
}
