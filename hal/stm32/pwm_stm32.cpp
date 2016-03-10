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

Pwm_stm32::Pwm_stm32(uint8_t id):
    id_(id),
    channel_id_(id / 2)
{
    if (id_ > 7)
    {
        id_         = 7;
        channel_id_ = 3;
    }

    pulse_us_[id_]  = 1500;
    period_us_[id_] = 20000;    // 50Hz

    //TODO adapt
    timer_peripheral_ = TIM1;
    prescaler_ = 168;
    period_ = 20000;
}

bool Pwm_stm32::init(void)
{
    bool success = true;
    int32_t gpio_success;

    /* Enable GPIOC clock. */
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_set_af(GPIOA, GPIO_AF1, GPIO8);
  
  /* Set GPIO12 (in GPIO port C) to 'output push-pull'. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF,
              GPIO_PUPD_NONE, GPIO8);

    // gpio_set(GPIOA, GPIO8);
    // gpio_clear(GPIOD, GPIO13);


    /* Enable TIM2 clock. */
    rcc_periph_clock_enable(RCC_TIM1);

    /* Enable TIM2 interrupt. */
    // nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    /* Reset TIM2 peripheral. */
    timer_reset(TIM1);

    /* Timer global mode:
     * - No divider
     * - Alignment edge
     * - Direction up
     */
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT,
               TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_set_prescaler(TIM1, prescaler_);

    /* Enable preload. */
    timer_disable_preload(TIM1);

    /* Continous mode. */
    timer_continuous_mode(TIM1);

    /* Period (36kHz). */
    timer_set_period(TIM1, period_);

    // Disable outputs. 
    timer_disable_oc_output(TIM1, TIM_OC1);
    // timer_disable_oc_output(TIM1, TIM_OC2);
    // timer_disable_oc_output(TIM1, TIM_OC3);
    // timer_disable_oc_output(TIM1, TIM_OC4);
    timer_disable_oc_clear(TIM1, TIM_OC1);
    timer_enable_oc_preload(TIM1, TIM_OC1);
    timer_set_oc_slow_mode(TIM1, TIM_OC1);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM1, TIM_OC1);
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_break_main_output(TIM1);
    /* -- OC1 configuration -- */

    /* Configure global mode of line 1. */
    timer_disable_oc_clear(TIM1, TIM_OC1);
    timer_disable_oc_preload(TIM1, TIM_OC1);
    timer_set_oc_slow_mode(TIM1, TIM_OC1);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_FROZEN);

    /* Set the capture compare value for OC1. */
    timer_set_oc_value(TIM1, TIM_OC1, 1000);

    /* ---- */

    /* ARR reload enable. */
    timer_enable_preload(TIM1);

    /* Counter enable. */
    timer_enable_counter(TIM1);

    /* Enable commutation interrupt. */
    // timer_enable_irq(TIM1, TIM_DIER_CC1IE);

    // /* init timer */
    // //enable timer clock
    // rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);

    // /* Reset TIM1 peripheral */
    // timer_reset(timer_peripheral_);

    // /* Set the timers global mode to:
    // * - use no divider
    // * - alignment edge
    // * - count direction up
    // */
    // timer_set_mode(timer_peripheral_,
    //             TIM_CR1_CKD_CK_INT,
    //             TIM_CR1_CMS_EDGE,
    //             TIM_CR1_DIR_UP);

    // timer_set_prescaler(timer_peripheral_, prescaler_);
    // timer_set_repetition_counter(timer_peripheral_, 0);
    // timer_enable_preload(timer_peripheral_);
    // timer_continuous_mode(timer_peripheral_);
    // timer_set_period(timer_peripheral_, period_);

    // /* init output channel */
    // /* Enable GPIO clock. */
    // rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);

    // /* Set timer channel to output */
    // gpio_mode_setup(    GPIOA, 
    //                     GPIO_MODE_AF,
    //                     GPIO_OTYPE_OD,
    //                     GPIO_TIM1_CH1);

    // timer_disable_oc_output(timer_peripheral_, TIM_OC1);
    // timer_set_oc_mode(timer_peripheral_, TIM_OC1, TIM_OCM_PWM1);
    // timer_set_oc_value(timer_peripheral_, TIM_OC1, 0);
    // timer_enable_oc_output(timer_peripheral_, TIM_OC1);

    // timer_set_oc_value(timer_peripheral_, TIM_OC1, pulse_us_[id_]);

    // timer_enable_counter(timer_peripheral_);

    return success;
}

bool Pwm_stm32::set_pulse_width_us(uint16_t pulse_us)
{
    pulse_us_[id_] = pulse_us;
    write_channel();

    return true;
}


bool Pwm_stm32::set_period_us(uint16_t period_us)
{
    period_us_[id_] = period_us;
    write_channel();

    return true;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Pwm_stm32::write_channel(void)
{
    // Set update frequency per channel with conservative method:
    // if two servos on the same channel ask for two different frequencies,
    // then the lowest frequecy is used
    // int32_t period  = max(period_us_[2 * channel_id_],
    //                       period_us_[2 * channel_id_ + 1]);

    // int32_t pulse_us_a = pulse_us_[2 * channel_id_];
    // int32_t pulse_us_b = pulse_us_[2 * channel_id_ + 1];
    // int32_t deadtime    = (period - pulse_us_a - pulse_us_b) / 2;

    // AVR32_PWM.channel[channel_id_ & 0b11].cprdupd   = period;
    // AVR32_PWM.channel[channel_id_ & 0b11].cdtyupd   = pulse_us_a + deadtime;
    // AVR32_PWM.channel[channel_id_ & 0b11].dtupd         = deadtime << 16 | deadtime;
}

// Allocate memory for static members here
uint32_t Pwm_stm32::pulse_us_[8];
uint32_t Pwm_stm32::period_us_[8];

void tim2_isr(void)
{
    if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {

        /* Clear compare interrupt flag. */
        timer_clear_flag(TIM2, TIM_SR_CC1IF);

        /* Toggle LED to indicate compare event. */
        gpio_clear(GPIOC, GPIO14);
    }
}
