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
 * \file i2c_chibios.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief I2C peripheral driver using ChibiOS/HAL
 *
 ******************************************************************************/

#ifndef I2C_CHIBIOS_HPP_
#define I2C_CHIBIOS_HPP_

#include "hal/common/i2c.hpp"

extern "C"
{
#include "hal.h"
}

/**
 * \brief   I2C peripheral driver using ChibiOS/HAL
 */
class I2c_chibios: public I2c
{
public:
    /**
     * \brief Configuration structure
     */
    struct conf_t
    {
        I2CDriver* driver;
        I2CConfig config;
        systime_t timeout;
    };


    /**
     * \brief   Constructor
     *
     * \param   config      Device configuration
     */
    I2c_chibios(conf_t config);


    /**
     * \brief       Hardware initialization
     *
     * \return  true Success
     * \return  false Error
     */
    bool init(void);


    /**
     * \brief   Test if a chip answers for a given I2C address
     *
     * \param   address     Slave adress
     *
     * \return  True        Slave found
     * \return  False       Slave not found
     */
    bool probe(uint32_t address);


    /**
     * \brief   Write multiple bytes to a I2C slave device
     *
     * \param   buffer      Data buffer
     * \param   nbytes      Number of bytes to write
     * \param   address     Slave adress
     *
     * \return  False       Data not written
     */
    bool write(const uint8_t* buffer, uint32_t nbytes, uint32_t address);


    /**
     * \brief   Read multiple bytes to a I2C slave device
     *
     * \param   buffer      Data buffer
     * \param   nbytes      Number of bytes to read
     * \param   address     Slave adress
     *
     * \return  True        Data successfully read
     * \return  False       Data not read
     */
    bool read(uint8_t* buffer, uint32_t nbytes, uint32_t address);

    /**
     * \brief   Write then Read data to/from an I2C device
     *
     * \param   out_buffer  Data buffer (output)
     * \param   ntxbytes    Number of bytes to write
     * \param   in_buffer   Data buffer (input)
     * \param   nrxbytes    Number of bytes to read
     *
     * \return  true        Success
     * \return  false       Failed
     */
    bool transfer(uint8_t* out_buffer, uint32_t ntxbytes, uint8_t* in_buffer, uint32_t nrxbytes, uint32_t address);

private:
    I2CDriver* driver_;   ///< I2C peripheral
    I2CConfig config_;    ///< Configuration
    systime_t timeout_;   ///< Timeout
};

#endif /* I2C_CHIBIOS_HPP_ */
