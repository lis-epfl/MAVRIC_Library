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
 * \file i2c_avr32.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief I2C peripheral driver for AVR32
 *
 ******************************************************************************/

#include "hal/avr32/i2c_avr32.hpp"
#include "hal/avr32/atmel_status_codes.hpp"

extern "C"
{
#include "util/print_util.h"
#include "libs/asf/avr32/drivers/gpio/gpio.h"
#include "libs/asf/common/services/clock/sysclk.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

I2c_avr32::I2c_avr32(i2c_avr32_conf_t config)
{
    config_ = config;
}


bool I2c_avr32::init(void)
{
    switch (config_.i2c_device)
    {
        case AVR32_I2C0:
            twim_ = &AVR32_TWIM0;
            ///< Register PDCA IRQ interrupt.
            // INTC_register_interrupt( (__int_handler) &i2c_int_handler_i2c0, AVR32_TWIM0_IRQ, AVR32_INTC_INT1);
            gpio_enable_module_pin(config_.clk_pin,
                                   AVR32_TWIMS0_TWCK_0_0_FUNCTION);
            gpio_enable_module_pin(config_.sda_pin,
                                   AVR32_TWIMS0_TWD_0_0_FUNCTION);

            break;
        case AVR32_I2C1:
            twim_ = &AVR32_TWIM1;///< Register PDCA IRQ interrupt.
            //INTC_register_interrupt( (__int_handler) &i2c_int_handler_i2c1, AVR32_TWIM1_IRQ, AVR32_INTC_INT1);
            gpio_enable_module_pin(config_.clk_pin,
                                   AVR32_TWIMS1_TWCK_0_0_FUNCTION);
            gpio_enable_module_pin(config_.sda_pin,
                                   AVR32_TWIMS1_TWD_0_0_FUNCTION);
            break;
        default: ///< invalid device ID
            return false;
    }

    gpio_enable_pin_pull_up(config_.clk_pin);
    gpio_enable_pin_pull_up(config_.sda_pin);

    config_.twi_opt.pba_hz = sysclk_get_pba_hz();

    status_code_t status;
    status = twim_master_init(twim_, &config_.twi_opt);

    return status_code_to_bool(status, true);
}


bool I2c_avr32::probe(uint32_t address)
{
    status_code_t status;
    //using 7bits addressing instead of 8bits R/W formating
    // status = twim_probe(twim_, address>>1);
    status = twim_probe(twim_, address);
    return status_code_to_bool(status);
}


bool I2c_avr32::write(const uint8_t* buffer, uint32_t nbytes, uint32_t address)
{
    status_code_t status;
    // atmel's implementation of TWIM uses 7bits address (no R/W bit), so we shift the 8 bit address
    // status = twim_write(twim_, buffer, nbytes, address>>1, config_.tenbit);
    status = twim_write(twim_, buffer, nbytes, address, config_.tenbit);
    return status_code_to_bool(status);
}


bool I2c_avr32::read(uint8_t* buffer, uint32_t nbytes, uint32_t address)
{
    status_code_t status;
    // atmel's implementation of TWIM uses 7bits address (no R/W bit), so we shift the 8 bit address
    // status = twim_read(twim_, buffer, nbytes, address>>1, config_.tenbit);
    status = twim_read(twim_, buffer, nbytes, address, config_.tenbit);
    return status_code_to_bool(status);
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
