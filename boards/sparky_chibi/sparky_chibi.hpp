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
 * \file    sparky_chibi.hpp
 *
 * \author  MAV'RIC Team
 * \author  Julien Lecoeur
 *
 * \brief   Autopilot for Sparky board based on STM32
 *
 ******************************************************************************/


#ifndef SPARKY_CHIBI_HPP_
#define SPARKY_CHIBI_HPP_


#include "drivers/barometer_ms5611.hpp"
#include "hal/common/led_gpio.hpp"
#include "drivers/state_display_sparky_v2.hpp"
#include "drivers/servo.hpp"

#include "hal/dummy/gpio_dummy.hpp"
#include "hal/dummy/pwm_dummy.hpp"
#include "hal/dummy/serial_dummy.hpp"

#include "hal/chibios/gpio_chibios.hpp"
#include "hal/chibios/i2c_chibios.hpp"
#include "hal/chibios/serial_chibios.hpp"
#include "hal/chibios/pwm_chibios.hpp"
#include "hal/chibios/spi_chibios.hpp"

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
// typedef struct
// {
    // gpio_stm32_conf_t        dsm_receiver_gpio_config;
    // gpio_stm32_conf_t        dsm_power_gpio_config;
    // gpio_stm32_conf_t        led_err_gpio_config;
    // gpio_stm32_conf_t        led_stat_gpio_config;
    // gpio_stm32_conf_t        led_rf_gpio_config;
    // Pwm_stm32::config_t      pwm_config[8];
    // Serial_usb_stm32::conf_t serial_usb_config;
    // servo_conf_t             servo_config[8];
    // spi_stm32_conf_t         spi_config[3];
    // i2c_stm32_conf_t         i2c_config[2];
    //
    // Barometer_MS5611::conf_t barometer_config;
// } sparky_v2_conf_t;


/**
 * \brief   Default configuration for the board
 *
 * \return  Config structure
 */
// static inline sparky_v2_conf_t sparky_v2_default_config();


/**
 * \brief  Boardsupport for the Sparky board
 *
 */
class Sparky_chibi
{
public:

    static const uint8_t PWM_COUNT    = 10;

    /**
     * \brief   Configuration structure
     */
    struct conf_t
    {
        Gpio_chibios::conf_t      gpio_led_err;
        Gpio_chibios::conf_t      gpio_led_stat;
        Gpio_chibios::conf_t      gpio_led_rf;
        Serial_chibios::conf_t    serial;
        Pwm_chibios::conf_t       pwm[PWM_COUNT];
        servo_conf_t              servo[PWM_COUNT];
        I2c_chibios::conf_t       i2c1;
        Barometer_MS5611::conf_t  barometer;
    };



    /**
     * \brief           Constructor
     *
     * \param   config  Board configuration
     */
    Sparky_chibi(conf_t config = default_config());


    /**
     * \brief   Hardware initialisation

     * \return  Success
     */
    bool init(void);


    /**
     * \brief   Default configuration for the board
     *
     * \return  Config structure
     */
    static inline conf_t default_config();


    /**
     * Public Members
     */
    // Gpio_stm32              led_err_gpio_;
    // Gpio_stm32              led_stat_gpio_;
    // Gpio_stm32              led_rf_gpio_;
    // Led_gpio                led_err_;
    // Led_gpio                led_stat_;
    // Led_gpio                led_rf_;
    // Pwm_stm32               pwm_0_;
    // Pwm_stm32               pwm_1_;
    // Pwm_stm32               pwm_2_;
    // Pwm_stm32               pwm_3_;
    // Pwm_stm32               pwm_4_;
    // Pwm_stm32               pwm_5_;
    // Pwm_dummy               pwm_6_;
    // Pwm_dummy               pwm_7_;
    // Serial_usb_stm32        serial_;
    // Servo                   servo_0_;
    // Servo                   servo_1_;
    // Servo                   servo_2_;
    // Servo                   servo_3_;
    // Servo                   servo_4_;
    // Servo                   servo_5_;
    // Servo                   servo_6_;
    // Servo                   servo_7_;
    // Spi_stm32               spi_1_;
    // Spi_stm32               spi_3_;
    // I2c_stm32               i2c_1_;
    // I2c_stm32               i2c_2_;
    // State_display_sparky_v2 state_display_sparky_v2_;



    Gpio_chibios            gpio_led_err_;
    Gpio_chibios            gpio_led_stat_;
    Gpio_chibios            gpio_led_rf_;
    Led_gpio                led_err_;
    Led_gpio                led_stat_;
    Led_gpio                led_rf_;
    Serial_chibios          serial_;
    Pwm_chibios             pwm_[PWM_COUNT];
    Servo                   servo_[PWM_COUNT];

    State_display_sparky_v2 state_display_;

    I2c_chibios             i2c1_;

    Barometer_MS5611        barometer_;

private:
    /**
     * /brief   Callback function that set PWM8 pin to high
     *
     * \detail  This is needed because PWM12 is not supported by CHibiOS, so we use PWM4 instead
     */
    static void pwmp8cb(PWMDriver *pwmp)
    {
      (void)pwmp;
      palSetPad(GPIOB, GPIOB_PIN15_PWM8);
    }

    /**
     * /brief   Callback function that set PWM8 pin to low
     *
     * \detail  This is needed because PWM12 is not supported by CHibiOS, so we use PWM4 instead
     */
    static void pwmp9cb(PWMDriver *pwmp)
    {
      (void)pwmp;
      palSetPad(GPIOB, GPIOB_PIN14_PWM9);
    }

    /**
     * /brief   Callback function that set PWM9 pin to high
     *
     * \detail  This is needed because PWM12 is not supported by CHibiOS, so we use PWM1 instead
     */
    static void pwmc9cb(PWMDriver *pwmp)
    {
      (void)pwmp;
      palClearPad(GPIOB, GPIOB_PIN14_PWM9);
    }

    /**
     * /brief   Callback function that set PWM9 pin to high
     *
     * \detail  This is needed because PWM12 is not supported by CHibiOS, so we use PWM1 instead
     */
    static void pwmc8cb(PWMDriver *pwmp)
    {
      (void)pwmp;
      palClearPad(GPIOB, GPIOB_PIN15_PWM8);
    }

    // byte_stream_t   dbg_stream_;  ///< Temporary member to make print_util work TODO: remove
};




/**
 * \brief   Default configuration for the board
 *
 * \return  Config structure
 */
Sparky_chibi::conf_t Sparky_chibi::default_config()
{
    conf_t conf = {};

    // -------------------------------------------------------------------------
    // GPIO config
    // -------------------------------------------------------------------------
    conf.gpio_led_err =
    {
        .port  = GPIOB,
        .pin   = GPIOB_PIN4
    };

    conf.gpio_led_stat =
    {
        .port  = GPIOB,
        .pin   = GPIOB_PIN5
    };

    conf.gpio_led_rf =
    {
        .port  = GPIOB,
        .pin   = GPIOB_PIN6
    };

    // -------------------------------------------------------------------------
    // Serial config
    // -------------------------------------------------------------------------
    conf.serial =
    {
        .id         = Serial_chibios::SERIAL_1,
        .device     = &UARTD1,
        .baudrate   = 38400
    };

    // -------------------------------------------------------------------------
    // PWM config
    // -------------------------------------------------------------------------
    // PWM12 is not directly supported by ChibiOS so we cannot use it for PWM8 and PWM9,
    // so we use
    // - PWM4 with callbacks pwmp8cb and pwmc8cb to toggle the pin GPIOB_PIN15_PWM8
    // - PWM1 with callbacks pwmp9cb and pwmc9cb to toggle the pin GPIOB_PIN14_PWM9
    //
    // Configuration of PWMs on the servo connectors:
    // ID Pin  Timer Channel
    // ---------------------
    // 0  PB0  TIM3  CH3
    // 1  PB1  TIM3  CH4
    // 2  PA3  TIM9  CH2
    // 3  PA2  TIM9  CH1
    // 4  PA1  TIM5  CH2
    // 5  PA0  TIM5  CH1
    //
    // Configuration of PWMs on the servo connectors:
    // ID Pin  Timer Channel
    // ---------------------
    // 6  PC9  TIM8  CH4
    // 7  PC8  TIM8  CH3
    // 8  PB15 TIM4  CH1 with callbacks
    // 9  PB14 TIM1  CH1 with callbacks

    for (size_t i = 0; i < PWM_COUNT; i++)
    {
        conf.pwm[i] = Pwm_chibios::default_config();
    }

    conf.pwm[0].driver  = &PWMD3;
    conf.pwm[0].channel = Pwm_chibios::CHANNEL_3;
    conf.pwm[1].driver  = &PWMD3;
    conf.pwm[1].channel = Pwm_chibios::CHANNEL_4;
    conf.pwm[2].driver  = &PWMD9;
    conf.pwm[2].channel = Pwm_chibios::CHANNEL_2;
    conf.pwm[3].driver  = &PWMD9;
    conf.pwm[3].channel = Pwm_chibios::CHANNEL_1;
    conf.pwm[4].driver  = &PWMD5;
    conf.pwm[4].channel = Pwm_chibios::CHANNEL_2;
    conf.pwm[5].driver  = &PWMD5;
    conf.pwm[5].channel = Pwm_chibios::CHANNEL_1;
    conf.pwm[6].driver  = &PWMD8;
    conf.pwm[6].channel = Pwm_chibios::CHANNEL_4;
    conf.pwm[7].driver  = &PWMD8;
    conf.pwm[7].channel = Pwm_chibios::CHANNEL_3;
    conf.pwm[8].driver  = &PWMD4;
    conf.pwm[8].channel = Pwm_chibios::CHANNEL_1;
    conf.pwm[8].config.callback = pwmp8cb;
    conf.pwm[8].config.channels[Pwm_chibios::CHANNEL_1].callback = &pwmc8cb;
    conf.pwm[9].driver  = &PWMD1;
    conf.pwm[9].channel = Pwm_chibios::CHANNEL_1;
    conf.pwm[9].config.callback = pwmp9cb;
    conf.pwm[9].config.channels[Pwm_chibios::CHANNEL_1].callback = &pwmc9cb;


    // -------------------------------------------------------------------------
    // Servos config
    // -------------------------------------------------------------------------
    for (size_t i = 0; i < PWM_COUNT; i++)
    {
        conf.servo[i] = servo_default_config_esc();
    }

    // -------------------------------------------------------------------------
    // I2C config
    // -------------------------------------------------------------------------
    conf.i2c1 =
    {
        .driver  = &I2CD1,
        .config  =
        {
            .op_mode     = OPMODE_I2C,
            .clock_speed = 400000,
            .duty_cycle  = FAST_DUTY_CYCLE_2
        },
        .timeout = 1000,
    };

    // -------------------------------------------------------------------------
    // Barometer config
    // -------------------------------------------------------------------------
    conf.barometer = Barometer_MS5611::default_config();

    // // -------------------------------------------------------------------------
    // // USB config
    // // -------------------------------------------------------------------------
    // conf.serial_usb_config = Serial_usb_stm32::default_config();
    //
    // // -------------------------------------------------------------------------
    // // SPI config
    // // -------------------------------------------------------------------------
    // conf.spi_config[0].spi_device       = STM32_SPI1;
    // conf.spi_config[0].mode             = STM32_SPI_IN_OUT;
    // conf.spi_config[0].clk_div          = SPI_CR1_BAUDRATE_FPCLK_DIV_128;
    //
    // conf.spi_config[0].miso_gpio_config.port    = GPIO_STM32_PORT_A;
    // conf.spi_config[0].miso_gpio_config.pin     = GPIO_STM32_PIN_6;
    // conf.spi_config[0].miso_gpio_config.dir     = GPIO_INPUT;
    // conf.spi_config[0].miso_gpio_config.pull    = GPIO_PULL_UPDOWN_DOWN;
    // conf.spi_config[0].miso_gpio_config.alt_fct = GPIO_STM32_AF_5;
    //
    // conf.spi_config[0].mosi_gpio_config.port    = GPIO_STM32_PORT_A;
    // conf.spi_config[0].mosi_gpio_config.pin     = GPIO_STM32_PIN_7;
    // conf.spi_config[0].mosi_gpio_config.dir     = GPIO_OUTPUT;
    // conf.spi_config[0].mosi_gpio_config.pull    = GPIO_PULL_UPDOWN_DOWN;
    // conf.spi_config[0].mosi_gpio_config.alt_fct = GPIO_STM32_AF_5;
    //
    // conf.spi_config[0].nss_gpio_config.port     = GPIO_STM32_PORT_A;
    // conf.spi_config[0].nss_gpio_config.pin      = GPIO_STM32_PIN_4;
    // conf.spi_config[0].nss_gpio_config.dir      = GPIO_OUTPUT;
    // conf.spi_config[0].nss_gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    // conf.spi_config[0].nss_gpio_config.alt_fct  = GPIO_STM32_AF_5;
    //
    // conf.spi_config[0].sck_gpio_config.port     = GPIO_STM32_PORT_A;
    // conf.spi_config[0].sck_gpio_config.pin      = GPIO_STM32_PIN_5;
    // conf.spi_config[0].sck_gpio_config.dir      = GPIO_OUTPUT;
    // conf.spi_config[0].sck_gpio_config.pull     = GPIO_PULL_UPDOWN_DOWN;
    // conf.spi_config[0].sck_gpio_config.alt_fct  = GPIO_STM32_AF_5;
    //
    //
    // conf.spi_config[2].spi_device               = STM32_SPI3;
    // conf.spi_config[2].mode                     = STM32_SPI_IN_OUT;
    // conf.spi_config[2].clk_div                  = SPI_CR1_BAUDRATE_FPCLK_DIV_64;
    //
    // conf.spi_config[2].miso_gpio_config.port    = GPIO_STM32_PORT_C;
    // conf.spi_config[2].miso_gpio_config.pin     = GPIO_STM32_PIN_11;
    // conf.spi_config[2].miso_gpio_config.dir     = GPIO_INPUT;
    // conf.spi_config[2].miso_gpio_config.pull    = GPIO_PULL_UPDOWN_DOWN;
    // conf.spi_config[2].miso_gpio_config.alt_fct = GPIO_STM32_AF_6;
    //
    // conf.spi_config[2].mosi_gpio_config.port    = GPIO_STM32_PORT_C;
    // conf.spi_config[2].mosi_gpio_config.pin     = GPIO_STM32_PIN_12;
    // conf.spi_config[2].mosi_gpio_config.dir     = GPIO_OUTPUT;
    // conf.spi_config[2].mosi_gpio_config.pull    = GPIO_PULL_UPDOWN_DOWN;
    // conf.spi_config[2].mosi_gpio_config.alt_fct = GPIO_STM32_AF_6;
    //
    // conf.spi_config[2].nss_gpio_config.port     = GPIO_STM32_PORT_A;
    // conf.spi_config[2].nss_gpio_config.pin      = GPIO_STM32_PIN_15;
    // conf.spi_config[2].nss_gpio_config.dir      = GPIO_OUTPUT;
    // conf.spi_config[2].nss_gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    // conf.spi_config[2].nss_gpio_config.alt_fct  = GPIO_STM32_AF_6;
    //
    // conf.spi_config[2].sck_gpio_config.port     = GPIO_STM32_PORT_C;
    // conf.spi_config[2].sck_gpio_config.pin      = GPIO_STM32_PIN_10;
    // conf.spi_config[2].sck_gpio_config.dir      = GPIO_OUTPUT;
    // conf.spi_config[2].sck_gpio_config.pull     = GPIO_PULL_UPDOWN_DOWN;
    // conf.spi_config[2].sck_gpio_config.alt_fct  = GPIO_STM32_AF_6;
    //
    // conf.i2c_config[0]                        = i2c_stm32_default_config();
    // conf.i2c_config[0].i2c_device_config      = STM32_I2C1;
    // conf.i2c_config[0].rcc_i2c_config         = RCC_I2C1;
    // conf.i2c_config[0].rcc_sda_port_config    = RCC_GPIOB;
    // conf.i2c_config[0].sda_config.port        = GPIO_STM32_PORT_B;
    // conf.i2c_config[0].sda_config.pin         = GPIO_STM32_PIN_9;
    // conf.i2c_config[0].sda_config.alt_fct     = GPIO_STM32_AF_4;
    // conf.i2c_config[0].rcc_clk_port_config    = RCC_GPIOB;
    // conf.i2c_config[0].clk_config.port        = GPIO_STM32_PORT_B;
    // conf.i2c_config[0].clk_config.pin         = GPIO_STM32_PIN_8;
    // conf.i2c_config[0].clk_config.alt_fct     = GPIO_STM32_AF_4;
    // conf.i2c_config[0].clk_speed              = 100000;
    // conf.i2c_config[0].tenbit_config          = false;  // currently only support 8 bits addressing
    // conf.i2c_config[0].timeout                = 20000;
    //
    // conf.i2c_config[1]                        = i2c_stm32_default_config();
    // conf.i2c_config[1].i2c_device_config      = STM32_I2C2;
    // conf.i2c_config[1].rcc_i2c_config         = RCC_I2C2;
    // conf.i2c_config[1].rcc_sda_port_config    = RCC_GPIOB;
    // conf.i2c_config[1].sda_config.port        = GPIO_STM32_PORT_B;
    // conf.i2c_config[1].sda_config.pin         = GPIO_STM32_PIN_11;
    // conf.i2c_config[1].sda_config.alt_fct     = GPIO_STM32_AF_4;
    // conf.i2c_config[1].rcc_clk_port_config    = RCC_GPIOB;
    // conf.i2c_config[1].clk_config.port        = GPIO_STM32_PORT_B;
    // conf.i2c_config[1].clk_config.pin         = GPIO_STM32_PIN_10;
    // conf.i2c_config[1].clk_config.alt_fct     = GPIO_STM32_AF_4;
    // conf.i2c_config[1].clk_speed              = 100000;
    // conf.i2c_config[1].tenbit_config          = false;  // currently only support 8 bits addressing
    // conf.i2c_config[1].timeout                = 20000;

    return conf;
}


#endif /* SPARKY_CHIBI_HPP_ */
