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
 * \file i2c_chibios.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief I2C peripheral driver using ChibiOS/HAL
 *
 ******************************************************************************/

#include "hal/chibios/i2c_chibios.hpp"

I2c_chibios::I2c_chibios(conf_t config):
    driver_(config.driver),
    config_(config.config),
    timeout_(config.timeout)
{
    ;
}


bool I2c_chibios::init(void)
{
    i2cStart(driver_, &config_);
    return true;
}


bool I2c_chibios::probe(uint32_t address)
{
    uint8_t data[1] = { 0 };
	return (write(data, 1, address));
}


bool I2c_chibios::write(const uint8_t* buffer, uint32_t nbytes, uint32_t address)
{
    msg_t status = MSG_OK;

    i2cAcquireBus(driver_);
    uint8_t rxbuf[1] = {0};
    status = i2cMasterTransmitTimeout(driver_, address, buffer, nbytes, rxbuf, 0, timeout_);
    i2cReleaseBus(driver_);

    return (status == MSG_OK);
}


bool I2c_chibios::read(uint8_t* buffer, uint32_t nbytes, uint32_t address)
{
    msg_t status = MSG_OK;

    i2cAcquireBus(driver_);
    status = i2cMasterReceiveTimeout(driver_, address, buffer, nbytes, timeout_);
    i2cReleaseBus(driver_);

    return (status == MSG_OK);
}


bool I2c_chibios::transfer(uint8_t* out_buffer, uint32_t ntxbytes, uint8_t* in_buffer, uint32_t nrxbytes, uint32_t address)
{
    msg_t status = MSG_OK;

    i2cAcquireBus(driver_);
    status = i2cMasterTransmitTimeout(driver_, address, out_buffer, ntxbytes, in_buffer, nrxbytes, timeout_);
    i2cReleaseBus(driver_);

    return (status == MSG_OK);
}
