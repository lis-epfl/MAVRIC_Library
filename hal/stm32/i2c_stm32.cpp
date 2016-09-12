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
 * \file i2c_stm32.cpp
 *
 * \author MAV'RIC Team
 * \author Gregoire HEITZ
 *
 * \brief I2C peripheral driver for STM32
 *
 ******************************************************************************/

#include "hal/stm32/i2c_stm32.hpp"

extern "C"
{
#include "util/print_util.hpp"
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
}


const uint32_t FLAG_MASK                = 0x00FFFFFF;
const uint32_t DATA_RECEIVED            = 0x00030040;
const uint32_t RECEIVER_MODE_SELECTED   = 0x00030002;

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
bool I2c_stm32::check_event(uint32_t i2c_event)
{
    uint32_t lastevent = 0;
    uint32_t flag1 = 0, flag2 = 0;

    //read the I2C status register
    flag1 = I2C_SR1(i2c_);
    flag2 = I2C_SR2(i2c_);
    flag2 = flag2 << 16;

    //get the last event
    lastevent = (flag1 | flag2) & (FLAG_MASK);

    if ( (lastevent & i2c_event) == i2c_event)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool I2c_stm32::start(uint8_t address, bool direction_is_transmit, bool ack)
{

    i2c_send_start(i2c_);
    //wait till not received
    uint16_t timeout = i2c_timeout_;
    while(!(I2C_SR1(i2c_) & I2C_SR1_SB))
    {
         if(--timeout == 0)
        {
            return false;
        }
    }

    //enable ack
    if (ack)
    {
        I2C_CR1(i2c_) |= I2C_CR1_ACK;
    }


    //send write/read bit
    if (direction_is_transmit)
    {
        //using 7bits addressing instead of 8bits R/W format
        I2C_DR(i2c_) = address<<1 & ~((uint16_t)0x0001);

        //wait till finished
        timeout = i2c_timeout_;
        while(!(I2C_SR1(i2c_) & I2C_SR1_ADDR))
        {
             if(--timeout == 0)
            {
                return false;
            }
        }
    }
    else
    {
        //using 7bits addressing instead of 8bits R/W format
        I2C_DR(i2c_) = address<<1 | ((uint16_t)0x0001);

        //wait till received
        timeout = i2c_timeout_;
        while(!(check_event(RECEIVER_MODE_SELECTED)))
        {
             if(--timeout == 0)
            {
                return false;
            }
        }
    }

    //Read status register to clear flag
    I2C_SR2(i2c_);

    return true;
}

bool I2c_stm32::stop(void)
{
    //wait till not busy anymore
    uint16_t timeout = i2c_timeout_;
    while( (!(I2C_SR1(i2c_) & I2C_SR1_TxE)) || (!(I2C_SR1(i2c_) & I2C_SR1_BTF)) )
    {
        if(--timeout == 0)
        {
            return false;
        }
    }

    I2C_CR1(i2c_) |= I2C_CR1_STOP;

    return true;
}


uint8_t I2c_stm32::read_ack(void)
{
    //enable ACK
    I2C_CR1(i2c_) |= I2C_CR1_ACK;

    //wait till received
    uint16_t timeout = i2c_timeout_;
    while(!(check_event(DATA_RECEIVED)))
    {
        if(--timeout == 0)
        {
            return 0;
        }
    }

    //read data
    return I2C_DR(i2c_);
}

uint8_t I2c_stm32::read_nack(void)
{
    //disable ACK
    I2C_CR1(i2c_) &= ~I2C_CR1_ACK;

    //generate stop
    I2C_CR1(i2c_) |= I2C_CR1_STOP;

    //wait till received
    uint16_t timeout = i2c_timeout_;
    while(!(check_event(DATA_RECEIVED)))
    {
        if(--timeout == 0)
        {
            return 0;
        }
    }

    //read data
    return I2C_DR(i2c_);
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

I2c_stm32::I2c_stm32(i2c_stm32_conf_t config)
{
    config_         = config;
    i2c_            = config.i2c_device_config;
    i2c_timeout_    = config.timeout;
}


bool I2c_stm32::init(void)
{
    bool init_success = true;

    // Enable RCC peripheral clock
    rcc_periph_clock_enable(config_.rcc_i2c_config);
    rcc_periph_clock_enable(config_.rcc_clk_port_config);
    rcc_periph_clock_enable(config_.rcc_sda_port_config);

    /* Setup GPIO pins for I2C transmit. */
    gpio_set_af(config_.clk_config.port, config_.clk_config.alt_fct, config_.clk_config.pin);
    gpio_set_af(config_.sda_config.port, config_.sda_config.alt_fct, config_.sda_config.pin);
    gpio_set_output_options(config_.clk_config.port, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, config_.clk_config.pin);
    gpio_set_output_options(config_.sda_config.port, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, config_.sda_config.pin);
    gpio_mode_setup(config_.clk_config.port,
                    GPIO_MODE_AF,
                    GPIO_PUPD_PULLUP,
                    config_.clk_config.pin);
    gpio_mode_setup(config_.sda_config.port,
                    GPIO_MODE_AF,
                    GPIO_PUPD_PULLUP,
                    config_.sda_config.pin);

    //set-up I2C
    i2c_peripheral_disable(i2c_);

    //clear freq
    I2C_CR2(i2c_) &= (uint16_t)~((uint16_t)(0x003F));
    //set freq
    I2C_CR2(i2c_) |= (uint16_t)(42);

    uint16_t result = 0;
    if (config_.clk_speed <= 100000)
    {
        //set maximum rise time
        I2C_TRISE(i2c_) = 42 + 1;

        //configure speed in fast mode
        result = (uint16_t)(42000000/(100000<<1));

        if (result < 0x04)
        {
            result = 0x04;
        }
    }
    else
    {
        //set maximum rise time
        I2C_TRISE(i2c_) = 12 + 1;

        //configure speed in fast mode
        result = (uint16_t)(42000000 / (400000 * 3));
        if ( (result & 0x0FFF) == 0)
        {
            //set min value allowed
            result |= (uint16_t)0x0001;
        }
    }

    //set speed value to fast mode
    I2C_CCR(i2c_) = (uint16_t)(result );

    //enable I2C device
    i2c_peripheral_enable(i2c_);

    //configure I2C_CR1(i2c_)
    //clear ACK, SMBTYPE and SMBUS
    // I2C_CR1(i2c_) &= ~(I2C_CR1_ACK | I2C_CR1_SMBTYPE | I2C_CR1_SMBUS);
    I2C_CR1(i2c_) &= 0xFBF5;
    // set i2c mode & no ack
    I2C_CR1(i2c_) |= uint16_t(0x0000|0x0000);

    //configure ORA1
    I2C_OAR1(i2c_) = (0x4000 | 0x0000);

    //enable I2C1
    i2c_peripheral_enable(i2c_);

    return init_success;
}


bool I2c_stm32::probe(uint32_t address)
{
    bool success = true;

    //if start succeed => device is connected
    success &= start(address, true, true);
    //TODO Pb with stop
    // success &= stop();

    return success;
}


bool I2c_stm32::write(const uint8_t* buffer, uint32_t nbytes, uint32_t address)
{
    bool success = true;

    //start
    success &= start(address, true, true);

    for (uint32_t i = 0; i < nbytes; ++i)
    {
        //wait till not busy anymore
        uint16_t timeout = i2c_timeout_;
        while(!(I2C_SR1(i2c_) & I2C_SR1_TxE))
        {
            if(--timeout == 0)
            {
                return false;
            }
        }

        //send i2c data
        I2C_DR(i2c_) = buffer[i];
    }

    success &= stop();

    return success;
}


bool I2c_stm32::read(uint8_t* buffer, uint32_t nbytes, uint32_t address)
{
    bool success = true;

    //Start
    success &= start(address, false, true);

     //read loop
    for (uint32_t i = 0; i < nbytes; ++i)
    {
        if(i == (nbytes-1))
        {
            buffer[i] = read_nack();
        }
        else
        {
            buffer[i] = read_ack();
        }
    }

    return success;
}
