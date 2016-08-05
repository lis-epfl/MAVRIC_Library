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
 * \file    spi_stm32.hpp
 *
 * \author  MAV'RIC Team
 * \author  Jean-Francois Burnier
 *
 * \brief   Implementation of spi peripheral for stm32
 *
 ******************************************************************************/

#ifndef SPI_STM32_HPP_
#define SPI_STM32_HPP_

#include "hal/common/spi.hpp"
#include "hal/stm32/gpio_stm32.hpp"

extern "C"
{
#include <libopencm3/stm32/spi.h>
}

/**
 * @brief   Enumerate the possible SPIs
 */
typedef enum
{
    STM32_SPI1             = SPI1,
    STM32_SPI2             = SPI2,
    STM32_SPI3             = SPI3,
} spi_stm32_devices_t;


/**
 * @brief   SPI modes
 */
typedef enum
{
    STM32_SPI_OFF       = 0,
    STM32_SPI_IN        = 1,
    STM32_SPI_OUT       = 2,
    STM32_SPI_IN_OUT    = 3,
} spi_stm32_mode_t;

/**
 * @brief   Configuration structure
 */
typedef struct
{
    spi_stm32_devices_t     spi_device;
    spi_stm32_mode_t        mode;
    uint8_t                 clk_div;            ///< fp clock division
    gpio_stm32_conf_t       miso_gpio_config;   ///< Master Out Slave In config
    gpio_stm32_conf_t       mosi_gpio_config;   ///< Master In Slave Out config
    gpio_stm32_conf_t       nss_gpio_config;    ///< Slave Select config
    gpio_stm32_conf_t       sck_gpio_config;    ///< Serial Clock config
} spi_stm32_conf_t;


/**
 * @brief   Implementation of spi peripheral for stm32
 */
class Spi_stm32: public Spi
{
public:

    /**
     * \brief   Initialises the peripheral
     *
     * \param   config      Device configuration
     */
    Spi_stm32(spi_stm32_conf_t spi_config);


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
    bool write(uint8_t* bytes, uint32_t nbytes = 1);


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
    bool read(uint8_t* in_buffer, uint32_t nbytes = 1);

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

    spi_stm32_conf_t            config_;        ///< Configuration
    spi_stm32_devices_t         spi_;           ///< Device ID

};


#endif /* SPI_STM32_HPP_ */