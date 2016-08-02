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

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Spi_stm32::Spi_stm32(spi_stm32_conf_t spi_config):
    config_(spi_config)
{}


bool Spi_stm32::init(void)
{
    uint32_t cr_tmp;
    bool     ret = true;

    // MISO init
    gpio_mode_setup(config_.miso_gpio_config.port, GPIO_MODE_AF, config_.miso_gpio_config.pull, config_.miso_gpio_config.pin);
    gpio_set_af(config_.miso_gpio_config.port, config_.miso_gpio_config.alt_fct, config_.miso_gpio_config.pin);
    //gpio_mode_setup(config_.miso_gpio_config.port, config_.miso_gpio_config.dir, config_.miso_gpio_config.pull, config_.miso_gpio_config.pin);

    // MOSI init
    gpio_mode_setup(config_.mosi_gpio_config.port, GPIO_MODE_AF, config_.mosi_gpio_config.pull, config_.mosi_gpio_config.pin);
    gpio_set_af(config_.mosi_gpio_config.port, config_.mosi_gpio_config.alt_fct, config_.mosi_gpio_config.pin);
    gpio_set_output_options(config_.mosi_gpio_config.port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, config_.mosi_gpio_config.pin);

    // SCK init
    gpio_mode_setup(config_.sck_gpio_config.port, GPIO_MODE_AF, config_.sck_gpio_config.pull, config_.sck_gpio_config.pin);
    gpio_set_af(config_.sck_gpio_config.port, config_.sck_gpio_config.alt_fct, config_.sck_gpio_config.pin);
    gpio_set_output_options(config_.sck_gpio_config.port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, config_.sck_gpio_config.pin);

    // NSS init
    gpio_set(config_.nss_gpio_config.port, config_.nss_gpio_config.pin);
    gpio_mode_setup(config_.nss_gpio_config.port, config_.nss_gpio_config.dir, config_.nss_gpio_config.pull, config_.nss_gpio_config.pin);

    cr_tmp =    SPI_CR1_BAUDRATE_FPCLK_DIV_8 |
                SPI_CR1_MSTR |
                SPI_CR1_SPE  |
                SPI_CR1_CPHA |
                SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE;

    switch (config_.spi_device)
    {
        case STM32_SPI1:
            rcc_periph_clock_enable(RCC_SPI1);
            SPI_CR2(SPI1) |= SPI_CR2_SSOE;
            SPI_CR1(SPI1)  = cr_tmp;
        break;

        case STM32_SPI2:
            rcc_periph_clock_enable(RCC_SPI2);
            SPI_CR2(SPI2) |= SPI_CR2_SSOE;
            SPI_CR1(SPI2)  = cr_tmp;
        break;

        case STM32_SPI3:
            rcc_periph_clock_enable(RCC_SPI3);
            SPI_CR2(SPI3) |= SPI_CR2_SSOE;
            SPI_CR1(SPI3)  = cr_tmp;
        break;

        default:
            ret = false;
    }

    return true;
}


bool Spi_stm32::write(uint8_t* bytes, uint32_t size)
{
    bool ret = true;

    return ret;
}


bool Spi_stm32::read(uint8_t* in_buffer, uint32_t nbytes)
{
    bool ret = true;

    return ret;
}

bool Spi_stm32::transfer(uint8_t* out_buffer, uint8_t* in_buffer, uint32_t nbytes)
{
    bool ret = true;

    return ret;
}

bool Spi_stm32::send(uint16_t value)
{
    //uint16_t value  = 0x01;
    uint16_t reg    = 0x02;

    bool     ret    = true;

    gpio_clear(config_.nss_gpio_config.port, config_.nss_gpio_config.pin); /* CS* select */

    switch (config_.spi_device)
    {
        case STM32_SPI1:
            spi_send(SPI1, reg);
            (void) spi_read(SPI1);
            spi_send(SPI1, value);
            (void) spi_read(SPI1);
        break;

        case STM32_SPI2:
            spi_send(SPI2, reg);
            (void) spi_read(SPI2);
            spi_send(SPI2, value);
            (void) spi_read(SPI2);
        break;

        case STM32_SPI3:
            spi_send(SPI3, reg);
            (void) spi_read(SPI3);
            spi_send(SPI3, value);
            (void) spi_read(SPI3);
        break;

        default:
            ret = false;
    }

    gpio_set(config_.nss_gpio_config.port, config_.nss_gpio_config.pin); /* CS* deselect */
    return ret;
}

uint16_t Spi_stm32::get(int reg)
{
    uint16_t d1 = 0, d2 = 0;

    bool     ret = true;

    gpio_clear(config_.nss_gpio_config.port, config_.nss_gpio_config.pin); /* CS* select */

    switch (config_.spi_device)
    {
        case STM32_SPI1:
            spi_send(SPI1, d1);
            d2 = spi_read(SPI1);
            d2 <<= 8;
            spi_send(SPI1, 0);
            d2 |= spi_read(SPI1);
        break;

        case STM32_SPI2:
            spi_send(SPI2, d1);
            d2 = spi_read(SPI2);
            d2 <<= 8;
            spi_send(SPI2, 0);
            d2 |= spi_read(SPI2);
        break;

        case STM32_SPI3:
            spi_send(SPI3, d1);
            d2 = spi_read(SPI3);
            d2 <<= 8;
            spi_send(SPI3, 0);
            d2 |= spi_read(SPI3);
        break;

        default:
            ret = false;
    }

    gpio_set(config_.nss_gpio_config.port, config_.nss_gpio_config.pin); /* CS* deselect */
    return d2;
}