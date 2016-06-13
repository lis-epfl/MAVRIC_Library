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
 * \file spektrum_satellite.c
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *
 * \brief Driver for spektrum satellite receiver
 *
 ******************************************************************************/


#include "drivers/spektrum_satellite.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
}

Spektrum_satellite* spek_sat;


//------------------------------------------------------------------------------
// SPECIAL FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * @brief           Glue function for interrupt handling
 *
 * @param serial    Peripheral
 */
void spektrum_irq_callback(Serial* serial)
{
    spek_sat->handle_interrupt();
};


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Spektrum_satellite::Spektrum_satellite(Serial& uart, Gpio& receiver_pin, Gpio& power_pin):
    uart_(uart),
    receiver_pin_(receiver_pin),
    power_pin_(power_pin),
    last_update_(0.0f)
{}

bool Spektrum_satellite::init(void)
{
    // Init pointer to sat
    spek_sat = this;

    bool result = true; //cannot go wrong...

    for (int32_t i = 0; i < 16; i++)
    {
        channels_[i] = 0;
    }

    protocol_           = RADIO_PROTOCOL_UNKNOWN;
    last_update_        = time_keeper_get_us();

    //Set minimum number of frames to be received in order to guess the radio protocol used
    protocol_proba_.min_nb_frames   = 10;
    protocol_proba_.proba_10bits    = 0;
    protocol_proba_.proba_11bits    = 0;

    // Attach interrupt handler function to uart
    result &= uart_.attach(spektrum_irq_callback);

    switch_on();

    return result;
}

void Spektrum_satellite::bind(radio_protocol_t protocol)
{
    int32_t i = 0;
    // uint32_t cpu_freq = sysclk_get_cpu_hz();

    print_util_dbg_print(" \n receive bind CMD \n");

    // Switch off satellite
    switch_off();
    time_keeper_delay_ms(100);

    //set as input, pull down not to be floating
    receiver_pin_.configure(GPIO_INPUT, GPIO_PULL_UPDOWN_DOWN);

    switch_on();

    // Wait for startup signal
    while ((receiver_pin_.read() == 0) && (i < 10000))
    {
        i++;
        time_keeper_delay_ms(1);
    }

    // Wait 100ms after receiver startup
    time_keeper_delay_ms(68);

    uint8_t pulses = 0;
    if (protocol == RADIO_PROTOCOL_DSM2_10BITS)
    {
        pulses = 3;
    }
    else if (protocol == RADIO_PROTOCOL_DSM2_11BITS)
    {
        pulses = 6;
    }

    // create 6 pulses with 250us period to set receiver to bind mode
    for (i = 0; i < pulses; i++)
    {
        receiver_pin_.configure(GPIO_OUTPUT, GPIO_PULL_UPDOWN_DOWN);
        time_keeper_delay_us(113);
        receiver_pin_.configure(GPIO_INPUT, GPIO_PULL_UPDOWN_UP);
        time_keeper_delay_us(118);
    }
}


int16_t Spektrum_satellite::channel(const uint8_t channel_number) const
{
    int16_t value;

    if (channel_number < 16)
    {
        value = channels_[channel_number];
    }
    else
    {
        value = 0;
    }

    return value;
}


uint32_t Spektrum_satellite::last_update(void) const
{
    return last_update_;
}


uint32_t Spektrum_satellite::dt(void) const
{
    return dt_;
}


void Spektrum_satellite::handle_interrupt(void)
{
    uint8_t c1 = 0;
    uint8_t c2 = 0;
    uint8_t i = 0;
    uint16_t sw;
    uint8_t channel;
    uint32_t now = time_keeper_get_us() ;

    // If byte received
    while (uart_.readable() > 0)
    {
        uint32_t dt_interrupt = now - last_interrupt_;
        last_interrupt_ = now;

        // the shorter frame period is 11'000us (11bits encoding) and the longer frame period is 22'000 us(10bits encoding)
        // the inter byte period within a frame is 77us
        // Clear buffer if the new byte of the on-going frame came after 2times the inter-byte period
        if ((receiver_.readable() != 0) && (dt_interrupt > 150))
        {
            receiver_.clear();
        }

        // Add new byte to buffer
        uart_.read(&c1);
        receiver_.put(c1);

        // If frame is complete, decode channels
        if (receiver_.readable() == 16)
        {
            if (protocol_ != RADIO_PROTOCOL_UNKNOWN)
            {
                // first two bytes are status info,
                receiver_.get(c1);
                receiver_.get(c2);

                if ((protocol_ == RADIO_PROTOCOL_DSM2_10BITS) && ((c1 != 0x03) || (c2 != 0xB2)))  //correspond to DSM2 10bits header
                {
                    receiver_.clear();
                    return;
                }

                for (i = 0; i < 7; i++) // 7 channels per frame
                {
                    receiver_.get(c1);
                    receiver_.get(c2);
                    sw = (uint16_t)c1 << 8 | ((uint16_t)c2);

                    if (protocol_ == RADIO_PROTOCOL_DSM2_10BITS)    //10 bits
                    {
                        // highest bit is frame 0/1, bits 2-6 are channel number
                        channel = ((sw >> 10)) & 0x0f;

                        // 10 bits per channel
                        channels_[channel] = ((int16_t)(sw & 0x3ff) - 512) * 2;
                    }
                    else if (protocol_ == RADIO_PROTOCOL_DSM2_11BITS)   //11bits
                    {
                        // highest bit is frame 0/1, bits 3-7 are channel number
                        channel = ((sw >> 11)) & 0x0f;

                        // 11 bits per channel
                        channels_[channel] = ((int16_t)(sw & 0x7ff) - 1024);
                    }
                }

                // Update timing
                dt_             = now - last_update_;
                last_update_    = now;
            }
            else
            {
                // The protocol is unknown => check the radio protocol

                // First two bytes are status info,
                receiver_.get(c1);
                receiver_.get(c2);

                // Increment probability for one of the protocols
                if (c1 == 0x03 && c2 == 0xB2)
                {
                    // Increment proba for DSM2_10BITS
                    protocol_proba_.proba_10bits++;

                    // Empty_the buffer, because we don't know the protocol yet
                    receiver_.clear();
                }
                else
                {
                    // Increment proba for DSM2_11BITS
                    protocol_proba_.proba_11bits++;

                    // Empty_the buffer, because we don't know the protocol yet
                    receiver_.clear();
                }

                // Check if enough frames were received
                if (protocol_proba_.min_nb_frames > 0)
                {
                    protocol_proba_.min_nb_frames--;
                }
                else
                {
                    // Once enough frames received, determine which protocol is used
                    if (protocol_proba_.proba_10bits > 2 * protocol_proba_.proba_11bits)
                    {
                        // DSM2_10BITS is more probable
                        protocol_ = RADIO_PROTOCOL_DSM2_10BITS;
                    }
                    else if (protocol_proba_.proba_11bits > 2 * protocol_proba_.proba_10bits)
                    {
                        // DSM2_11BITS is more probable
                        protocol_ = RADIO_PROTOCOL_DSM2_11BITS;
                    }
                    else
                    {
                        // No clear result => redo this probability check for 10 other frames
                        protocol_proba_.min_nb_frames = 10;
                    }
                }
            }
        }
    }
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Spektrum_satellite::switch_on(void)
{
    power_pin_.configure(GPIO_OUTPUT, GPIO_PULL_UPDOWN_NONE);
    power_pin_.set_low();
}


void Spektrum_satellite::switch_off(void)
{
    power_pin_.configure(GPIO_OUTPUT, GPIO_PULL_UPDOWN_NONE);
    power_pin_.set_high();
}
