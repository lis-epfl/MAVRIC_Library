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
 * \file    spi_chibios.hpp
 *
 * \author  MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   SPI peripherals driver using CHibios/HAL
 *
 ******************************************************************************/

#include "hal/chibios/spi_chibios.hpp"


Spi_chibios::Spi_chibios(conf_t config):
    driver_(config.driver),
    config_(config.config)
{}


bool Spi_chibios::init(void)
{
    spiAcquireBus(driver_);
    spiStart(driver_, &config_);
    spiReleaseBus(driver_);
    return true;
}


bool Spi_chibios::write(uint8_t* out_buffer, uint32_t nbytes)
{
    spiAcquireBus(driver_);
    spiStart(driver_, &config_);
    spiSend(driver_, nbytes, out_buffer);
    // spiUnselect(driver_);
    spiReleaseBus(driver_);
    return true;
}


bool Spi_chibios::read(uint8_t* in_buffer, uint32_t nbytes)
{
    spiAcquireBus(driver_);
    spiStart(driver_, &config_);
    spiReceive(driver_, nbytes, in_buffer);
    // spiUnselect(driver_);
    spiReleaseBus(driver_);
    return true;
}


bool Spi_chibios::transfer(uint8_t* out_buffer, uint8_t* in_buffer, uint32_t nbytes)
{
    spiAcquireBus(driver_);
    spiStart(driver_, &config_);
    spiExchange(driver_, nbytes, out_buffer, in_buffer);
    // spiUnselect(driver_);
    spiReleaseBus(driver_);
    return true;
}
