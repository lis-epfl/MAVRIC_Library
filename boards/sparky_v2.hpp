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
 * \file    sparky_v2.hpp
 *
 * \author  MAV'RIC Team
 * \author  Jean-Francois Burnier
 *
 * \brief   Autopilot for Sparky board based on STM32
 *
 ******************************************************************************/


#ifndef SPARKY_V2_HPP_
#define SPARKY_V2_HPP_

#include "hal/stm32/gpio_stm32.hpp"
#include "hal/stm32/i2c_stm32.hpp"
#include "hal/stm32/pwm_stm32.hpp"
#include "hal/stm32/serial_stm32.hpp"

#include "drivers/airspeed_analog.hpp"
#include "drivers/battery.hpp"
#include "drivers/servo.hpp"
#include "drivers/sonar_i2cxl.hpp"
#include "drivers/spektrum_satellite.hpp"
#include "drivers/state_display_sparky_v2.hpp"

#include "simulation/dynamic_model_quad_diag.hpp"
#include "simulation/simulation.hpp"

#include "hal/dummy/serial_dummy.hpp"
#include "hal/dummy/i2c_dummy.hpp"
#include "hal/dummy/adc_dummy.hpp"
#include "hal/dummy/file_dummy.hpp"
#include "hal/dummy/pwm_dummy.hpp"
#include "hal/dummy/gpio_dummy.hpp"
#include "hal/common/led_gpio.hpp"

extern "C"
{
#include "util/streams.h"
}


// Preprocessor definitions

/*
 * Should the ESC be calibrated?
 * 0 for false (normal flight)
 * 1 for true (calibration)
 * !!!IMPORTANT!!!
 * IF CALIBRATING, TAKE OFF PROPS
 */
#define CALIBRATE_ESC 0


/**
 * \brief   Configuration structure
 */
typedef struct
{
    gpio_stm32_conf_t       dsm_receiver_gpio_config;
    gpio_stm32_conf_t       dsm_power_gpio_config;
    gpio_stm32_conf_t       led_err_gpio_config;
    gpio_stm32_conf_t       led_stat_gpio_config;
    gpio_stm32_conf_t       led_rf_gpio_config;
    //serial_stm32_conf_t     serial_1_config;
    //serial_stm32_conf_t     serial_2_config;
} sparky_v2_conf_t;


/**
 * \brief   Default configuration for the board
 *
 * \return  Config structure
 */
static inline sparky_v2_conf_t sparky_v2_default_config();


/**
 * \brief  Boardsupport for the Sparky board
 *
 */
class Sparky_v2
{
public:
    /**
     * \brief           Constructor
     *
     * \param   config  Board configuration
     */
    Sparky_v2(sparky_v2_conf_t config = sparky_v2_default_config());


    /**
     * \brief   Hardware initialisation

     * \return  Success
     */
    bool init(void);

    /**
     * Public Members
     */
    Gpio_stm32              led_err_gpio_;
    Gpio_stm32              led_stat_gpio_;
    Gpio_stm32              led_rf_gpio_;
    Led_gpio                led_err_;
    Led_gpio                led_stat_;
    Led_gpio                led_rf_;
    State_display_sparky_v2 state_display_sparky_v2_;
    //File_dummy              file_flash;
    //Serial_stm32            serial_1;
    //Serial_stm32            serial_2;

private:
    byte_stream_t   dbg_stream_;  ///< Temporary member to make print_util work TODO: remove
};


/**
 * \brief   Default configuration for the board
 *
 * \return  Config structure
 */
static inline sparky_v2_conf_t sparky_v2_default_config()
{
    sparky_v2_conf_t conf = {};

    // -------------------------------------------------------------------------
    // GPIO config
    // -------------------------------------------------------------------------

    // Error Led
    conf.led_err_gpio_config.port    = GPIO_STM32_PORT_B;
    conf.led_err_gpio_config.pin     = GPIO_STM32_PIN_4;
    conf.led_err_gpio_config.dir     = GPIO_OUTPUT;
    conf.led_err_gpio_config.pull    = GPIO_PULL_UPDOWN_NONE;

    // Status Led
    conf.led_stat_gpio_config.port   = GPIO_STM32_PORT_B;
    conf.led_stat_gpio_config.pin    = GPIO_STM32_PIN_5;
    conf.led_stat_gpio_config.dir    = GPIO_OUTPUT;
    conf.led_stat_gpio_config.pull   = GPIO_PULL_UPDOWN_NONE;

    // Rf Led
    conf.led_rf_gpio_config.port     = GPIO_STM32_PORT_B;
    conf.led_rf_gpio_config.pin      = GPIO_STM32_PIN_6;
    conf.led_rf_gpio_config.dir      = GPIO_OUTPUT;
    conf.led_rf_gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;

    /*
    // -------------------------------------------------------------------------
    // Serial config
    // -------------------------------------------------------------------------
    conf.serial_1_config                = serial_stm32_default_config();
    conf.serial_1_config.device         = SERIAL_STM32_4;
    conf.serial_1_config.baudrate       = 57600;
    conf.serial_1_config.databits       = SERIAL_STM32_DATABITS_8;
    conf.serial_1_config.stopbits       = SERIAL_STM32_STOPBITS_1;
    conf.serial_1_config.parity         = SERIAL_STM32_PARITY_NONE;
    conf.serial_1_config.mode           = SERIAL_STM32_MODE_TX_RX;
    conf.serial_1_config.flow_control   = SERIAL_STM32_FLOWCONTROL_NONE;
    conf.serial_1_config.rx_port        = GPIO_STM32_PORT_A;
    conf.serial_1_config.rx_pin         = GPIO_STM32_PIN_1;
    conf.serial_1_config.rx_af          = GPIO_STM32_AF_8;
    conf.serial_1_config.tx_port        = GPIO_STM32_PORT_A;
    conf.serial_1_config.tx_pin         = GPIO_STM32_PIN_0;
    conf.serial_1_config.tx_af          = GPIO_STM32_AF_8;

    // -------------------------------------------------------------------------
    // Serial config
    // -------------------------------------------------------------------------
    conf.serial_2_config                = serial_stm32_default_config();
    conf.serial_2_config.device         = SERIAL_STM32_2;
    conf.serial_2_config.baudrate       = 115200;
    conf.serial_2_config.databits       = SERIAL_STM32_DATABITS_8;
    conf.serial_2_config.stopbits       = SERIAL_STM32_STOPBITS_1;
    conf.serial_2_config.parity         = SERIAL_STM32_PARITY_NONE;
    conf.serial_2_config.mode           = SERIAL_STM32_MODE_TX_RX;
    conf.serial_2_config.flow_control   = SERIAL_STM32_FLOWCONTROL_NONE;
    conf.serial_2_config.rx_port        = GPIO_STM32_PORT_A;
    conf.serial_2_config.rx_pin         = GPIO_STM32_PIN_3;
    conf.serial_2_config.rx_af          = GPIO_STM32_AF_7;
    conf.serial_2_config.tx_port        = GPIO_STM32_PORT_A;
    conf.serial_2_config.tx_pin         = GPIO_STM32_PIN_2;
    conf.serial_2_config.tx_af          = GPIO_STM32_AF_7;
    */

    return conf;
}


#endif /* SPARKY_HPP_ */