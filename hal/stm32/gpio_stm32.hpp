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
 * \file    gpio_stm32.hpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Implementation of GPIO peripherals for STM32
 *
 ******************************************************************************/

#ifndef GPIO_STM32_H_
#define GPIO_STM32_H_

#include <libopencm3/stm32/gpio.h>
#include "hal/common/gpio.hpp"
#include <cstdint>


typedef enum
{
    GPIO_STM32_PORT_A = GPIOA,
    GPIO_STM32_PORT_B = GPIOB,
    GPIO_STM32_PORT_C = GPIOC,
    GPIO_STM32_PORT_D = GPIOD,
    GPIO_STM32_PORT_E = GPIOE,
    GPIO_STM32_PORT_F = GPIOF,
} gpio_stm32_port_t;


typedef enum
{
    GPIO_STM32_PIN_0  = GPIO0,
    GPIO_STM32_PIN_1  = GPIO1,
    GPIO_STM32_PIN_2  = GPIO2,
    GPIO_STM32_PIN_3  = GPIO3,
    GPIO_STM32_PIN_4  = GPIO4,
    GPIO_STM32_PIN_5  = GPIO5,
    GPIO_STM32_PIN_6  = GPIO6,
    GPIO_STM32_PIN_7  = GPIO7,
    GPIO_STM32_PIN_8  = GPIO8,
    GPIO_STM32_PIN_9  = GPIO9,
    GPIO_STM32_PIN_10 = GPIO10,
    GPIO_STM32_PIN_11 = GPIO11,
    GPIO_STM32_PIN_12 = GPIO12,
    GPIO_STM32_PIN_13 = GPIO13,
    GPIO_STM32_PIN_14 = GPIO14,
    GPIO_STM32_PIN_15 = GPIO15,
} gpio_stm32_pin_t;


typedef enum
{
    GPIO_STM32_AF_0     = GPIO_AF0 ,
    GPIO_STM32_AF_1     = GPIO_AF1 ,
    GPIO_STM32_AF_2     = GPIO_AF2 ,
    GPIO_STM32_AF_3     = GPIO_AF3 ,
    GPIO_STM32_AF_4     = GPIO_AF4 ,
    GPIO_STM32_AF_5     = GPIO_AF5 ,
    GPIO_STM32_AF_6     = GPIO_AF6 ,
    GPIO_STM32_AF_7     = GPIO_AF7 ,
    GPIO_STM32_AF_8     = GPIO_AF8 ,
    GPIO_STM32_AF_9     = GPIO_AF9 ,
    GPIO_STM32_AF_10    = GPIO_AF10,
    GPIO_STM32_AF_11    = GPIO_AF11,
    GPIO_STM32_AF_12    = GPIO_AF12,
    GPIO_STM32_AF_13    = GPIO_AF13,
    GPIO_STM32_AF_14    = GPIO_AF14,
    GPIO_STM32_AF_15    = GPIO_AF15,
} gpio_stm32_alt_function_t;


/**
 *  Configuration structure
 */
typedef struct
{
    gpio_stm32_port_t           port;           ///< port number
    gpio_stm32_pin_t            pin;            ///< pin number
    gpio_dir_t                  dir;            ///< Direction
    gpio_pull_updown_t          pull;           ///< Pull up/down
    gpio_stm32_alt_function_t   alt_fct;        ///< Alternate function
} gpio_stm32_conf_t;


/**
 * \brief   Default configuration
 *
 * \return  Config structure
 */
static inline gpio_stm32_conf_t gpio_stm32_default_config();


class Gpio_stm32: public Gpio
{
public:
    /**
     * \brief   Initialises the peripheral
     *
     * \param   config      Device configuration
     */
    Gpio_stm32(gpio_stm32_conf_t config = gpio_stm32_default_config());


    /**
     * \brief   Hardware initialization
     *
     * \return  true        Success
     * \return  false       Error
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
     * \brief   Write 1 to the gpio
     *
     * \return  true        Success
     * \return  false       Failed
     */
    bool set_high(void);


    /**
     * \brief   Write 0 to the gpio
     *
     * \return  true        Success
     * \return  false       Failed
     */
    bool set_low(void);


    /**
     * \brief   Toggle the gpio value
     * \details Writes 0 if currently high, writes 1 if currently low
     *
     * \return  true        Success
     * \return  false       Failed
     */
    bool toggle(void);


    /**
     * \brief   Write to the gpio pin
     *
     * \param   level       Value to write
     *
     * \return  true        Success
     * \return  false       Failed
     */
    bool write(bool level);


    /**
     * \brief   Read the current gpio level
     *
     * \return  Level
     */
    bool read(void);

private:
    gpio_stm32_conf_t   config_;    ///< Device configuration
};


/**
 * \brief   Default configuration
 *
 * \return  Config structure
 */
static inline gpio_stm32_conf_t gpio_stm32_default_config()
{
    gpio_stm32_conf_t conf = {};
    conf.port   = GPIO_STM32_PORT_A;
    conf.pin    = GPIO_STM32_PIN_0;
    conf.dir    = GPIO_INPUT;
    conf.pull   = GPIO_PULL_UPDOWN_NONE;
    return conf;
}

#endif /* GPIO_STM32_H_ */