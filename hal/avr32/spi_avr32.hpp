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
 * \file    spi_avr32.hpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Implementation of spi peripheral for avr32
 *
 ******************************************************************************/

#ifndef SPI_AVR32_HPP_
#define SPI_AVR32_HPP_

#include "hal/common/spi.hpp"

extern "C"
{
#include "libs/asf/avr32/drivers/spi/spi.h"
}

/**
 * @brief   Enumerate the possible UARTs
 */
typedef enum
{
    AVR32_SPI_0             = 0,
    AVR32_SPI_1             = 1,
    AVR32_SPI_2             = 2,
    AVR32_SPI_3             = 3,
    AVR32_SPI_4             = 4,
    AVR32_SPI_MAX_NUMBER    = 5,
} spi_avr32_devices_t;


/**
 * @brief   UART modes
 */
typedef enum
{
    AVR32_SPI_OFF       = 0,
    AVR32_SPI_IN        = 1,
    AVR32_SPI_OUT       = 2,
    AVR32_SPI_IN_OUT    = 3,
} spi_avr32_mode_t;


/**
 * @brief   Gpio map
 */
typedef struct
{
    uint8_t  pin;                       ///< Module pin.
    uint8_t  function;                  ///< Module function.
} spi_avr32_gpio_map_t;

/**
 * @brief   Configuration structure
 */
typedef struct
{
    spi_avr32_devices_t     spi_device;
    spi_avr32_mode_t        mode;
    spi_options_t           options;
    spi_avr32_gpio_map_t    mosi_pin_map;   ///< Master Out Slave In pin
    spi_avr32_gpio_map_t    miso_pin_map;   ///< Master In Slave Out pin
    spi_avr32_gpio_map_t    sck_pin_map;    ///< Serial Clock pin
    spi_avr32_gpio_map_t    ss_pin_map;     ///< Slave Select pin
} spi_avr32_conf_t;


/**
 * @brief   Implementation of spi peripheral for avr32
 */
class Spi_avr32: public Spi
{
public:

    /**
     * \brief   Initialises the peripheral
     *
     * \param   config      Device configuration
     */
    Spi_avr32(spi_avr32_conf_t config);


    /**
     * \brief   Hardware initialization
     *
     * \return  true Success
     * \return  false Error
     */
    bool init(void);


    /**
     * \brief   Write bytes on the spi line
     *
     * \param   byte        Outgoing bytes
     * \param   size        Number of bytes to write
     *
     * \return  true        Data successfully written
     * \return  false       Data not written
     */
    bool write(const uint8_t* bytes, const uint32_t size = 1);


    /**
     * \brief   Read bytes from the spi line
     *
     * \param   command     Reading command
     * \param   bytes       Incoming bytes
     * \param   size        Number of bytes to read
     *
     * \return  true        Data successfully read
     * \return  false       Data not read
     */
    bool read(uint16_t command, uint8_t* bytes, const uint32_t size = 1);

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
    spi_avr32_conf_t            config_;        ///< Configuration
    volatile avr32_spi_t*       spi_;           ///< Hardware peripheral

};


#endif /* SPI_AVR32_HPP_ */