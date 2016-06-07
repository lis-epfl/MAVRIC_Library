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
 * \file i2c_stm32.hpp
 *
 * \author MAV'RIC Team
 * \author Gregoire HEITZ
 *
 * \brief I2C peripheral driver for STM32
 *
 ******************************************************************************/

#ifndef I2C_STM32_H_
#define I2C_STM32_H_

#include "hal/common/i2c.hpp"
#include "hal/stm32/gpio_stm32.hpp"

extern "C"
{
    #include <libopencm3/stm32/i2c.h>
    #include <libopencm3/stm32/rcc.h>
}


/**
 * \brief   Enumerate the 3 possible I2C
 */
typedef enum
{
    STM32_I2C1 = I2C1,
    STM32_I2C2 = I2C2,
    STM32_I2C3 = I2C3
} i2c_stm32_devices_t;


/**
 * @brief   Configuration structure
 */
typedef struct
{
    i2c_stm32_devices_t i2c_device_config;      ///< I2C device number
    bool                tenbit_config;          ///< using 7 or 10 bits addressing
    gpio_stm32_conf_t   sda_config;             ///< sda port settings
    gpio_stm32_conf_t   clk_config;             ///< clk port settings
    rcc_periph_clken    rcc_i2c_config;         ///< corresponding i2c for rcc
    rcc_periph_clken    rcc_sda_port_config;    ///< corresponding port for rcc
    rcc_periph_clken    rcc_clk_port_config;    ///< corresponding port for rcc
    uint32_t            clk_speed;              ///< i2c clk speed
    uint16_t            timeout;                ///< i2c timeout
} i2c_stm32_conf_t;

/**
 * \brief   Default configuration
 *
 * \return  Config structure
 */
static inline  i2c_stm32_conf_t i2c_stm32_default_config();

/**
 * @brief   I2C peripheral driver for STM32
 */
class I2c_stm32: public I2c
{
public:
    /**
     * @brief   Initialises the peripheral
     *
     * @param   config      Device configuration
     */
    I2c_stm32(i2c_stm32_conf_t config = i2c_stm32_default_config());


    /**
     * @brief       Hardware initialization
     *
     * @return  true Success
     * @return  false Error
     */
    bool init(void);


    /**
     * @brief   Test if a chip answers for a given I2C address
     *
     * @param   address     Slave adress
     *
     * @return  True        Slave found
     * @return  False       Slave not found
     */
    bool probe(uint32_t address);


    /**
     * @brief   Write multiple bytes to a I2C slave device
     *
     * @param   buffer      Data buffer
     * @param   nbytes      Number of bytes to write
     * @param   address     Slave adress
     *
     * @return  True        Data successfully written
     * @return  False       Data not written
     */
    bool write(const uint8_t* buffer, uint32_t nbytes, uint32_t address);


    /**
     * @brief   Read multiple bytes to a I2C slave device
     *
     * @param   buffer      Data buffer
     * @param   nbytes      Number of bytes to read
     * @param   address     Slave adress
     *
     * @return  True        Data successfully read
     * @return  False       Data not read
     */
    bool read(uint8_t* buffer, uint32_t nbytes, uint32_t address);
    

private:
    i2c_stm32_conf_t        config_;        ///< Configuration
    i2c_stm32_devices_t     i2c_;           ///< I2C device
    uint16_t                i2c_timeout_;   ///< I2C timeout

    /**
     * @brief   Check if an i2c event occured
     *
     * @param   i2c_event   Type of I2C event
     *
     * @return  True        event occured
     * @return  False       event didn't occur
     */
    bool check_event(uint32_t i2c_event);

    /**
     * @brief   Start I2C communication
     *
     * @param   address                 Slave adress
     * @param   direction_is_transmit   Write (true) Read (false)
     * @param   ack                     Using Acknowledgment or not
     *
     * @return  starting communication succeed or not
     */
    bool start(uint8_t address, bool direction_is_transmit, bool ack);

    /**
     * @brief   Test if a chip answers for a given I2C address
     *
     * @param   address     Slave adress
     *
     * @return  stopping communication succeed or not
     */
    bool stop(void);

    /**
     * @brief   Read a byte and Acknowledge its reception
     *
     * @return  byte read
     */
    uint8_t read_ack(void);

    /**
     * @brief   Read a byte without acknowledge its reception
     *
     * @return  byte read
     */
    uint8_t read_nack(void);
};

static inline  i2c_stm32_conf_t i2c_stm32_default_config()
{
    i2c_stm32_conf_t conf = {};
    conf.i2c_device_config  = STM32_I2C2;
    conf.rcc_i2c_config     = RCC_I2C2;
    conf.rcc_sda_port_config     = RCC_GPIOB;
    conf.sda_config.port    = GPIO_STM32_PORT_B;
    conf.sda_config.pin     = GPIO_STM32_PIN_11;
    conf.sda_config.alt_fct = GPIO_STM32_AF_4;
    conf.rcc_clk_port_config     = RCC_GPIOB;
    conf.clk_config.port    = GPIO_STM32_PORT_B;
    conf.clk_config.pin     = GPIO_STM32_PIN_10;
    conf.clk_config.alt_fct = GPIO_STM32_AF_4;
    conf.clk_speed          = 400000;
    conf.tenbit_config      = false;
    conf.timeout            = 20000;
    return conf;
}


#endif /* I2C_STM32_H_ */