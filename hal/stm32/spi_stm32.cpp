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
 * \file    spi_stm32.cpp
 *
 * \author  MAV'RIC Team
 * \author  Jean-Francois Burnier
 *
 * \brief   Implementation of spi peripheral for stm32
 *
 ******************************************************************************/

#include "hal/stm32/spi_stm32.hpp"
#include <cstddef>

extern "C"
{
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Spi_stm32::Spi_stm32(spi_stm32_conf_t spi_config):
    config_(spi_config),
    spi_(spi_config.spi_device)
{}


bool Spi_stm32::init(void)
{
    bool     ret = true;

    // SPI IOs configurations

    // MISO init
    gpio_mode_setup(config_.miso_gpio_config.port, GPIO_MODE_AF, config_.miso_gpio_config.pull, config_.miso_gpio_config.pin);
    gpio_set_af(config_.miso_gpio_config.port, config_.miso_gpio_config.alt_fct, config_.miso_gpio_config.pin);

    // MOSI init
    gpio_mode_setup(config_.mosi_gpio_config.port, GPIO_MODE_AF, config_.mosi_gpio_config.pull, config_.mosi_gpio_config.pin);
    gpio_set_af(config_.mosi_gpio_config.port, config_.mosi_gpio_config.alt_fct, config_.mosi_gpio_config.pin);
    gpio_set_output_options(config_.mosi_gpio_config.port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, config_.mosi_gpio_config.pin);

    // SCK init
    gpio_mode_setup(config_.sck_gpio_config.port, GPIO_MODE_AF, config_.sck_gpio_config.pull, config_.sck_gpio_config.pin);
    gpio_set_af(config_.sck_gpio_config.port, config_.sck_gpio_config.alt_fct, config_.sck_gpio_config.pin);
    gpio_set_output_options(config_.sck_gpio_config.port, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, config_.sck_gpio_config.pin);

    // NSS init
    if (config_.ss_mode_hard)
    {
        gpio_set(config_.nss_gpio_config.port, config_.nss_gpio_config.pin);
        gpio_mode_setup(config_.nss_gpio_config.port, GPIO_MODE_OUTPUT, config_.nss_gpio_config.pull, config_.nss_gpio_config.pin);
        gpio_set_output_options(config_.nss_gpio_config.port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, config_.nss_gpio_config.pin);
    }

    gpio_set(GPIO_STM32_PORT_B, GPIO_STM32_PIN_3);
    gpio_mode_setup(GPIO_STM32_PORT_B, GPIO_OUTPUT, GPIO_PULL_UPDOWN_NONE, GPIO_STM32_PIN_3);

    // SPI configuration

    switch (config_.spi_device)
    {
        case STM32_SPI1:
            rcc_periph_clock_enable(RCC_SPI1);
        break;

        case STM32_SPI2:
            rcc_periph_clock_enable(RCC_SPI2);
        break;

        case STM32_SPI3:
            rcc_periph_clock_enable(RCC_SPI3);
        break;

        default:
            ret = false;
    }

    if (config_.ss_mode_hard)
    {
        spi_disable_software_slave_management(spi_);
        spi_enable_ss_output(spi_);
    }
    else
        spi_enable_software_slave_management(spi_);

    SPI_CR1(spi_) |= config_.clk_div;           // Clock frequency
    spi_set_standard_mode(spi_, config_.mode);
    spi_enable(spi_);
    spi_set_master_mode(spi_);

    return ret;
}


bool Spi_stm32::write(uint8_t* bytes, uint32_t nbytes)
{
    bool ret;

    ret = transfer(bytes, NULL, nbytes);

    return ret;
}


bool Spi_stm32::read(uint8_t* in_buffer, uint32_t nbytes)
{
    bool ret;

    ret = transfer(NULL, in_buffer, nbytes);

    return ret;
}

bool Spi_stm32::transfer(uint8_t* out_buffer, uint8_t* in_buffer, uint32_t nbytes)
{
    bool ret = true;

    // slave select
    if (config_.ss_mode_hard)
        gpio_clear(config_.nss_gpio_config.port, config_.nss_gpio_config.pin);
    else
        spi_set_nss_high(spi_);

    if ((out_buffer != NULL) && (in_buffer != NULL))
    {
        // TX & RX transfer
        for (uint32_t i = 0; i < nbytes; i++)
        {
            in_buffer[i] = spi_xfer(spi_, out_buffer[i]);
        }
    }
    else if ((out_buffer == NULL) && (in_buffer != NULL))
    {
        // RX transfer
        for (uint32_t i = 0; i < nbytes; i++)
        {
            in_buffer[i] = spi_xfer(spi_, 0);
        }
    }
    else if ((out_buffer != NULL) && (in_buffer == NULL))
    {
        // TX transfer
        for (uint32_t i = 0; i < nbytes; i++)
        {
            (void)spi_xfer(spi_, out_buffer[i]);
        }
    }
    else if ((out_buffer == NULL) && (in_buffer == NULL))
    {
        // Error
        ret = false;
    }

    // slave deselect
    if (config_.ss_mode_hard)
        gpio_set(config_.nss_gpio_config.port, config_.nss_gpio_config.pin);
    else
        spi_set_nss_low(spi_);

    return ret;
}
