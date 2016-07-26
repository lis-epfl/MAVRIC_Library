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
 * \file    gpio_avr32.hpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Implementation of GPIO peripherals for avr32
 *
 ******************************************************************************/

#ifndef GPIO_AVR32_HPP_
#define GPIO_AVR32_HPP_

#include "hal/common/gpio.hpp"
#include <cstdint>

/**
 *  Configuration structure
 */
typedef struct
{
    uint32_t pin;           ///< pin number
    gpio_dir_t dir;
    gpio_pull_updown_t pull;
} gpio_avr32_conf_t;


/**
 * @brief   Default configuration
 *
 * @return  Config structure
 */
static inline gpio_avr32_conf_t gpio_avr32_default_config();


class Gpio_avr32: public Gpio
{
public:
    /**
     * @brief   Initialises the peripheral
     *
     * @param   config      Device configuration
     */
    Gpio_avr32(gpio_avr32_conf_t config = gpio_avr32_default_config());


    /**
     * @brief   Hardware initialization
     *
     * @return  true        Success
     * @return  false       Error
     */
    bool init(void);


    /**
     * \brief   Configures the GPIO
     *
     * \param   dir     Pin direction (one of enum gpio_dir_t)
     * \param   pull    Pin pull up/down (one of enum gpio_pull_updown_t)
     *
     * \return  success
     */
    bool configure(gpio_dir_t dir, gpio_pull_updown_t pull);


    /**
     * @brief   Write 1 to the gpio
     *
     * @return  true        Success
     * @return  false       Failed
     */
    bool set_high(void);


    /**
     * @brief   Write 0 to the gpio
     *
     * @return  true        Success
     * @return  false       Failed
     */
    bool set_low(void);


    /**
     * @brief   Toggle the gpio value
     * @details Writes 0 if currently high, writes 1 if currently low
     *
     * @return  true        Success
     * @return  false       Failed
     */
    bool toggle(void);


    /**
     * @brief   Write to the gpio pin
     *
     * @param   level       Value to write
     *
     * @return  true        Success
     * @return  false       Failed
     */
    bool write(bool level);


    /**
     * @brief   Read the current gpio level
     *
     * @return  Level
     */
    bool read(void);

private:
    gpio_avr32_conf_t   config_;    ///< Device configuration
};


/**
 * @brief   Default configuration
 *
 * @return  Config structure
 */
static inline gpio_avr32_conf_t gpio_avr32_default_config()
{
    gpio_avr32_conf_t conf = {};
    conf.pin    = 0;
    conf.dir    = GPIO_INPUT;
    conf.pull   = GPIO_PULL_UPDOWN_NONE;

    return conf;
}

#endif /* GPIO_AVR32_HPP_ */