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

#ifndef SPI_CHIBIOS_HPP_
#define SPI_CHIBIOS_HPP_

#include "hal/common/spi.hpp"

extern "C"
{
#include "hal.h"
}


/**
 * \brief   SPI peripherals driver using CHibios/HAL
 */
class Spi_chibios: public Spi
{
public:
    /**
     * \brief Configuration structure
     */
    struct conf_t
    {
        SPIDriver* driver;
        SPIConfig config;
    };

    /**
     * \brief   Constructor
     *
     * \param   config      Device configuration
     */
    Spi_chibios(conf_t config);

    /**
     * \brief   Hardware initialization
     *
     * \return  true        Success
     * \return  false       Error
     */
    bool init(void);


    /**
     * \brief   Write data to the SPI bus
     *
     * \param   out_buffer  Data buffer
     * \param   nbytes      Number of bytes to write
     *
     * \return  true        Success
     * \return  false       Failed
     */
    bool write(uint8_t* out_buffer, uint32_t nbytes);


    /**
     * \brief   Read data from the SPI bus
     *
     * \param   in_buffer   Data buffer
     * \param   nbytes      Number of bytes to read
     *
     * \return  true        Success
     * \return  false       Failed
     */
    bool read(uint8_t* in_buffer, uint32_t nbytes);


    /**
     * \brief   Write and Read data to/from the SPI bus
     *
     * \param   out_buffer  Data buffer (output)
     * \param   in_buffer   Data buffer (input)
     * \param   nbytes      Number of bytes to write/read
     *
     * \return  true        Success
     * \return  false       Failed
     */
    bool transfer(uint8_t* out_buffer, uint8_t* in_buffer, uint32_t nbytes);

private:
    SPIDriver* driver_;   ///< SPI peripheral
    SPIConfig config_;    ///< Configuration
};


#endif /* SPI_CHIBIOS_HPP_ */
