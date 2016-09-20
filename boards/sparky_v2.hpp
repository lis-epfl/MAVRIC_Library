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

#include "hal/dummy/pwm_dummy.hpp"

#include "hal/stm32/gpio_stm32.hpp"
#include "hal/stm32/pwm_stm32.hpp"
#include "hal/stm32/serial_stm32.hpp"
#include "hal/stm32/serial_usb_stm32.hpp"
#include "hal/stm32/spi_stm32.hpp"
#include "hal/stm32/i2c_stm32.hpp"

#include "drivers/mpu_9250.hpp"
#include "drivers/servo.hpp"
#include "drivers/state_display_sparky_v2.hpp"
#include "drivers/barometer_ms5611.hpp"

#include "hal/dummy/serial_dummy.hpp"
#include "hal/dummy/gpio_dummy.hpp"
#include "hal/common/led_gpio.hpp"

#include "sensing/imu.hpp"

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
 * \brief  Boardsupport for the Sparky board
 *
 */
class Sparky_v2
{
public:

    static const uint8_t PWM_COUNT = 10;

    /**
     * \brief   Configuration structure
     */
    struct conf_t
    {
        gpio_stm32_conf_t           dsm_receiver_gpio_config;   ///< DSM reveiver GPIO configuration
        gpio_stm32_conf_t           dsm_power_gpio_config;      ///< DSM power GPIO configuration
        gpio_stm32_conf_t           led_err_gpio_config;        ///< Error led GPIO configuration
        gpio_stm32_conf_t           led_stat_gpio_config;       ///< Status led GPIO configuration
        gpio_stm32_conf_t           led_rf_gpio_config;         ///< Rf led GPIO configuration
        gpio_stm32_conf_t           nss_gpio_config[3];         ///< Slave Select configuration
        imu_conf_t                  imu_config;                 ///< IMU configuration
        Pwm_stm32::config_t         pwm_config[PWM_COUNT];      ///< PWM configuration
        serial_stm32_conf_t         serial_1_config;            ///< Serial configuration
        Serial_usb_stm32::conf_t    serial_usb_config;          ///< Serial USB configuration
        servo_conf_t                servo_config[PWM_COUNT];    ///< Servo configuration
        spi_stm32_conf_t            spi_config[2];              ///< SPI configuration
    };

    /**
     * \brief   Default configuration for the board
     *
     * \return  Config structure
     */
    static inline conf_t default_config();


    /**
     * \brief           Constructor
     *
     * \param   config  Board configuration
     */
    Sparky_v2(conf_t config = default_config());


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
    Serial_stm32            serial_1_;
    Serial_usb_stm32        serial_;
    Pwm_stm32               pwm_[PWM_COUNT];
    Servo                   servo_[PWM_COUNT];
    Spi_stm32               spi_1_;
    Spi_stm32               spi_3_;
    Gpio_stm32              nss_1_gpio_;
    Gpio_stm32              nss_2_gpio_;
    Gpio_stm32              nss_3_gpio_;
    State_display_sparky_v2 state_display_sparky_v2_;
    Mpu_9250                mpu_9250_;
    Imu                     imu_;

private:
    byte_stream_t   dbg_stream_;  ///< Temporary member to make print_util work TODO: remove
};


/**
 * \brief   Default configuration for the board
 *
 * \return  Config structure
 */
Sparky_v2::conf_t Sparky_v2::default_config()
{
    conf_t conf = {};

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

    // -------------------------------------------------------------------------
    // Imu config
    // -------------------------------------------------------------------------
    conf.imu_config = imu_default_config();
    // Accelerometer

    // Axis and sign
    conf.imu_config.accelerometer.sign[0] = -1.0f;  ///< +1 or -1
    conf.imu_config.accelerometer.sign[1] = -1.0f;
    conf.imu_config.accelerometer.sign[2] = -1.0f;
    conf.imu_config.accelerometer.axis[0] = 1;      ///< Should be 0, 1, or 2
    conf.imu_config.accelerometer.axis[1] = 0;
    conf.imu_config.accelerometer.axis[2] = 2;

    // Gyroscope

    // Axis and sign
    conf.imu_config.gyroscope.sign[0] = -1.0f;  ///< +1 or -1
    conf.imu_config.gyroscope.sign[1] = -1.0f;
    conf.imu_config.gyroscope.sign[2] = -1.0f;
    conf.imu_config.gyroscope.axis[0] = 1;      ///< Should be 0, 1, or 2
    conf.imu_config.gyroscope.axis[1] = 0;
    conf.imu_config.gyroscope.axis[2] = 2;

    // Magnetometer

    // Scale
    conf.imu_config.magnetometer.scale_factor[0] = 250.0f;      ///< Should be >0
    conf.imu_config.magnetometer.scale_factor[1] = 250.0f;
    conf.imu_config.magnetometer.scale_factor[2] = 300.0f;

    // Axis and sign
    conf.imu_config.magnetometer.sign[0] = -1.0f;   ///< +1 or -1
    conf.imu_config.magnetometer.sign[1] = -1.0f;
    conf.imu_config.magnetometer.sign[2] = +1.0f;
    conf.imu_config.magnetometer.axis[0] = 0;       ///< Should be 0, 1, or 2
    conf.imu_config.magnetometer.axis[1] = 1;
    conf.imu_config.magnetometer.axis[2] = 2;

    // -------------------------------------------------------------------------
    // PWM config
    // -------------------------------------------------------------------------
    // Warning the PWM configuration is different from the one on the Sparky_V2 datasheet
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
    // 8  PB15 TIM12 CH2
    // 9  PB14 TIM12 CH1

    conf.pwm_config[0].gpio_config.port     = GPIO_STM32_PORT_B;
    conf.pwm_config[0].gpio_config.pin      = GPIO_STM32_PIN_0;
    conf.pwm_config[0].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[0].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[0].gpio_config.alt_fct  = GPIO_STM32_AF_2;
    conf.pwm_config[0].timer                = TIM3;
    conf.pwm_config[0].rcc_timer            = RCC_TIM3;
    conf.pwm_config[0].channel              = Pwm_stm32::PWM_STM32_CHANNEL_3;
    conf.pwm_config[0].prescaler            = 84;    // max timer clock freq of TIM3 is 84MHz
    conf.pwm_config[0].period               = 20000; // 50Hz
    conf.pwm_config[0].pulse_us             = 5000;

    conf.pwm_config[1].gpio_config.port     = GPIO_STM32_PORT_B;
    conf.pwm_config[1].gpio_config.pin      = GPIO_STM32_PIN_1;
    conf.pwm_config[1].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[1].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[1].gpio_config.alt_fct  = GPIO_STM32_AF_2;
    conf.pwm_config[1].timer                = TIM3;
    conf.pwm_config[1].rcc_timer            = RCC_TIM3;
    conf.pwm_config[1].channel              = Pwm_stm32::PWM_STM32_CHANNEL_4;
    conf.pwm_config[1].prescaler            = 84;    // max timer clock freq of TIM3 is 84MHz
    conf.pwm_config[1].period               = 20000; // 50Hz
    conf.pwm_config[1].pulse_us             = 5000;

    conf.pwm_config[2].gpio_config.port     = GPIO_STM32_PORT_A;
    conf.pwm_config[2].gpio_config.pin      = GPIO_STM32_PIN_3;
    conf.pwm_config[2].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[2].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[2].gpio_config.alt_fct  = GPIO_STM32_AF_3;
    conf.pwm_config[2].timer                = TIM9;
    conf.pwm_config[2].rcc_timer            = RCC_TIM9;
    conf.pwm_config[2].channel              = Pwm_stm32::PWM_STM32_CHANNEL_2;
    conf.pwm_config[2].prescaler            = 168;   // max timer clock freq of TIM9 is 168MHz
    conf.pwm_config[2].period               = 20000; // 50Hz
    conf.pwm_config[2].pulse_us             = 5000;

    conf.pwm_config[3].gpio_config.port     = GPIO_STM32_PORT_A;
    conf.pwm_config[3].gpio_config.pin      = GPIO_STM32_PIN_2;
    conf.pwm_config[3].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[3].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[3].gpio_config.alt_fct  = GPIO_STM32_AF_3;
    conf.pwm_config[3].timer                = TIM9;
    conf.pwm_config[3].rcc_timer            = RCC_TIM9;
    conf.pwm_config[3].channel              = Pwm_stm32::PWM_STM32_CHANNEL_1;
    conf.pwm_config[3].prescaler            = 168;   // max timer clock freq of TIM9 is 168MHz
    conf.pwm_config[3].period               = 20000; // 50Hz
    conf.pwm_config[3].pulse_us             = 5000;

    conf.pwm_config[4].gpio_config.port     = GPIO_STM32_PORT_A;
    conf.pwm_config[4].gpio_config.pin      = GPIO_STM32_PIN_1;
    conf.pwm_config[4].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[4].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[4].gpio_config.alt_fct  = GPIO_STM32_AF_2;
    conf.pwm_config[4].timer                = TIM5;
    conf.pwm_config[4].rcc_timer            = RCC_TIM5;
    conf.pwm_config[4].channel              = Pwm_stm32::PWM_STM32_CHANNEL_2;
    conf.pwm_config[4].prescaler            = 84;    // max timer clock freq of TIM5 is 84MHz
    conf.pwm_config[4].period               = 20000; // 50Hz
    conf.pwm_config[4].pulse_us             = 5000;

    conf.pwm_config[5].gpio_config.port     = GPIO_STM32_PORT_A;
    conf.pwm_config[5].gpio_config.pin      = GPIO_STM32_PIN_0;
    conf.pwm_config[5].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[5].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[5].gpio_config.alt_fct  = GPIO_STM32_AF_2;
    conf.pwm_config[5].timer                = TIM5;
    conf.pwm_config[5].rcc_timer            = RCC_TIM5;
    conf.pwm_config[5].channel              = Pwm_stm32::PWM_STM32_CHANNEL_1;
    conf.pwm_config[5].prescaler            = 84;    // max timer clock freq of TIM5 is 84MHz
    conf.pwm_config[5].period               = 20000; // 50Hz
    conf.pwm_config[5].pulse_us             = 5000;

    conf.pwm_config[9].gpio_config.port     = GPIO_STM32_PORT_C;
    conf.pwm_config[9].gpio_config.pin      = GPIO_STM32_PIN_9;
    conf.pwm_config[9].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[9].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[9].gpio_config.alt_fct  = GPIO_STM32_AF_2;
    conf.pwm_config[9].timer                = TIM8;
    conf.pwm_config[9].rcc_timer            = RCC_TIM8;
    conf.pwm_config[9].channel              = Pwm_stm32::PWM_STM32_CHANNEL_4;
    conf.pwm_config[9].prescaler            = 84;    // max timer clock freq of TIM8 is 84MHz
    conf.pwm_config[9].period               = 20000; // 50Hz
    conf.pwm_config[9].pulse_us             = 5000;

    conf.pwm_config[8].gpio_config.port     = GPIO_STM32_PORT_C;
    conf.pwm_config[8].gpio_config.pin      = GPIO_STM32_PIN_8;
    conf.pwm_config[8].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[8].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[8].gpio_config.alt_fct  = GPIO_STM32_AF_3;
    conf.pwm_config[8].timer                = TIM8;
    conf.pwm_config[8].rcc_timer            = RCC_TIM8;
    conf.pwm_config[8].channel              = Pwm_stm32::PWM_STM32_CHANNEL_3;
    conf.pwm_config[8].prescaler            = 84;    // max timer clock freq of TIM8 is 84MHz
    conf.pwm_config[8].period               = 20000; // 50Hz
    conf.pwm_config[8].pulse_us             = 5000;

    conf.pwm_config[7].gpio_config.port     = GPIO_STM32_PORT_B;
    conf.pwm_config[7].gpio_config.pin      = GPIO_STM32_PIN_15;
    conf.pwm_config[7].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[7].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[7].gpio_config.alt_fct  = GPIO_STM32_AF_9;
    conf.pwm_config[7].timer                = TIM12;
    conf.pwm_config[7].rcc_timer            = RCC_TIM12;
    conf.pwm_config[7].channel              = Pwm_stm32::PWM_STM32_CHANNEL_2;
    conf.pwm_config[7].prescaler            = 84;    // max timer clock freq of TIM12 is 84MHz
    conf.pwm_config[7].period               = 20000; // 50Hz
    conf.pwm_config[7].pulse_us             = 5000;

    conf.pwm_config[6].gpio_config.port     = GPIO_STM32_PORT_B;
    conf.pwm_config[6].gpio_config.pin      = GPIO_STM32_PIN_14;
    conf.pwm_config[6].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[6].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[6].gpio_config.alt_fct  = GPIO_STM32_AF_9;
    conf.pwm_config[6].timer                = TIM12;
    conf.pwm_config[6].rcc_timer            = RCC_TIM12;
    conf.pwm_config[6].channel              = Pwm_stm32::PWM_STM32_CHANNEL_1;
    conf.pwm_config[6].prescaler            = 84;    // max timer clock freq of TIM12 is 84MHz
    conf.pwm_config[6].period               = 20000; // 50Hz
    conf.pwm_config[6].pulse_us             = 5000;


    // -------------------------------------------------------------------------
    // Servo config
    // -------------------------------------------------------------------------
    // conf.servo_config[0] = servo_default_config_brush_motor();
    // // with a 2S battery we need to divide the duty cycle by 2 not to over-heat
    // conf.servo_config[0].pulse_center_us    = 62;
    // conf.servo_config[0].pulse_magnitude_us = 62;  // duty cycle: 0 to 100%
    // conf.servo_config[1] = servo_default_config_brush_motor();
    // conf.servo_config[1].pulse_center_us    = 62;
    // conf.servo_config[1].pulse_magnitude_us = 62;  // duty cycle: 0 to 100%
    // conf.servo_config[2] = servo_default_config_brush_motor();
    // conf.servo_config[2].pulse_center_us    = 62;
    // conf.servo_config[2].pulse_magnitude_us = 62;  // duty cycle: 0 to 100%
    // conf.servo_config[3] = servo_default_config_brush_motor();
    // conf.servo_config[3].pulse_center_us    = 62;
    // conf.servo_config[3].pulse_magnitude_us = 62;  // duty cycle: 0 to 100%
    // conf.servo_config[4] = servo_default_config_esc();
    // conf.servo_config[5] = servo_default_config_esc();
    // conf.servo_config[6] = servo_default_config_esc();
    // conf.servo_config[7] = servo_default_config_esc();
    for (size_t i = 0; i < 8; i++)
    {
        conf.servo_config[i] = servo_default_config_esc();
    }

    // -------------------------------------------------------------------------
    // Serial 1 config
    // -------------------------------------------------------------------------
    conf.serial_1_config                = serial_stm32_default_config();
    conf.serial_1_config.device         = SERIAL_STM32_1;
    conf.serial_1_config.baudrate       = 57600;
    conf.serial_1_config.databits       = SERIAL_STM32_DATABITS_8;
    conf.serial_1_config.stopbits       = SERIAL_STM32_STOPBITS_1;
    conf.serial_1_config.parity         = SERIAL_STM32_PARITY_NONE;
    conf.serial_1_config.mode           = SERIAL_STM32_MODE_TX_RX;
    conf.serial_1_config.flow_control   = SERIAL_STM32_FLOWCONTROL_NONE;
    conf.serial_1_config.rx_port        = GPIO_STM32_PORT_A;
    conf.serial_1_config.rx_pin         = GPIO_STM32_PIN_10;
    conf.serial_1_config.rx_af          = GPIO_STM32_AF_7;
    conf.serial_1_config.tx_port        = GPIO_STM32_PORT_A;
    conf.serial_1_config.tx_pin         = GPIO_STM32_PIN_9;
    conf.serial_1_config.tx_af          = GPIO_STM32_AF_7;

    // -------------------------------------------------------------------------
    // USB config
    // -------------------------------------------------------------------------
    conf.serial_usb_config = Serial_usb_stm32::default_config();

    // -------------------------------------------------------------------------
    // SPI config
    // -------------------------------------------------------------------------

    // SPI 1 config
    conf.spi_config[0].spi_device               = STM32_SPI1;
    conf.spi_config[0].mode                     = STM32_SPI_MODE_CPOL1_CPHA1;
    conf.spi_config[0].clk_div                  = SPI_CR1_BAUDRATE_FPCLK_DIV_128;
    conf.spi_config[0].ss_mode_hard             = true;

    conf.spi_config[0].miso_gpio_config.port    = GPIO_STM32_PORT_A;
    conf.spi_config[0].miso_gpio_config.pin     = GPIO_STM32_PIN_6;
    conf.spi_config[0].miso_gpio_config.dir     = GPIO_INPUT;
    conf.spi_config[0].miso_gpio_config.pull    = GPIO_PULL_UPDOWN_UP;
    conf.spi_config[0].miso_gpio_config.alt_fct = GPIO_STM32_AF_5;

    conf.spi_config[0].mosi_gpio_config.port    = GPIO_STM32_PORT_A;
    conf.spi_config[0].mosi_gpio_config.pin     = GPIO_STM32_PIN_7;
    conf.spi_config[0].mosi_gpio_config.dir     = GPIO_OUTPUT;
    conf.spi_config[0].mosi_gpio_config.pull    = GPIO_PULL_UPDOWN_UP;
    conf.spi_config[0].mosi_gpio_config.alt_fct = GPIO_STM32_AF_5;

    conf.spi_config[0].sck_gpio_config.port     = GPIO_STM32_PORT_A;
    conf.spi_config[0].sck_gpio_config.pin      = GPIO_STM32_PIN_5;
    conf.spi_config[0].sck_gpio_config.dir      = GPIO_OUTPUT;
    conf.spi_config[0].sck_gpio_config.pull     = GPIO_PULL_UPDOWN_UP;
    conf.spi_config[0].sck_gpio_config.alt_fct  = GPIO_STM32_AF_5;

    // SPI 1 slave select config
    conf.nss_gpio_config[0].port                   = GPIO_STM32_PORT_C;
    conf.nss_gpio_config[0].pin                    = GPIO_STM32_PIN_4;
    conf.nss_gpio_config[0].dir                    = GPIO_OUTPUT;
    conf.nss_gpio_config[0].pull                   = GPIO_PULL_UPDOWN_UP;

    // SPI 3 config
    conf.spi_config[1].spi_device               = STM32_SPI3;
    conf.spi_config[1].mode                     = STM32_SPI_MODE_CPOL1_CPHA1;
    conf.spi_config[1].clk_div                  = SPI_CR1_BAUDRATE_FPCLK_DIV_64;
    conf.spi_config[1].ss_mode_hard             = true;

    conf.spi_config[1].miso_gpio_config.port    = GPIO_STM32_PORT_C;
    conf.spi_config[1].miso_gpio_config.pin     = GPIO_STM32_PIN_11;
    conf.spi_config[1].miso_gpio_config.dir     = GPIO_INPUT;
    conf.spi_config[1].miso_gpio_config.pull    = GPIO_PULL_UPDOWN_DOWN;
    conf.spi_config[1].miso_gpio_config.alt_fct = GPIO_STM32_AF_6;

    conf.spi_config[1].mosi_gpio_config.port    = GPIO_STM32_PORT_C;
    conf.spi_config[1].mosi_gpio_config.pin     = GPIO_STM32_PIN_12;
    conf.spi_config[1].mosi_gpio_config.dir     = GPIO_OUTPUT;
    conf.spi_config[1].mosi_gpio_config.pull    = GPIO_PULL_UPDOWN_DOWN;
    conf.spi_config[1].mosi_gpio_config.alt_fct = GPIO_STM32_AF_6;

    conf.spi_config[1].sck_gpio_config.port     = GPIO_STM32_PORT_C;
    conf.spi_config[1].sck_gpio_config.pin      = GPIO_STM32_PIN_10;
    conf.spi_config[1].sck_gpio_config.dir      = GPIO_OUTPUT;
    conf.spi_config[1].sck_gpio_config.pull     = GPIO_PULL_UPDOWN_DOWN;
    conf.spi_config[1].sck_gpio_config.alt_fct  = GPIO_STM32_AF_6;

    // SPI 3 slaves select config
    conf.nss_gpio_config[1].port                   = GPIO_STM32_PORT_A;
    conf.nss_gpio_config[1].pin                    = GPIO_STM32_PIN_15;
    conf.nss_gpio_config[1].dir                    = GPIO_OUTPUT;
    conf.nss_gpio_config[1].pull                   = GPIO_PULL_UPDOWN_NONE;

    conf.nss_gpio_config[2].port                   = GPIO_STM32_PORT_B;
    conf.nss_gpio_config[2].pin                    = GPIO_STM32_PIN_3;
    conf.nss_gpio_config[2].dir                    = GPIO_OUTPUT;
    conf.nss_gpio_config[2].pull                   = GPIO_PULL_UPDOWN_NONE;

    return conf;
}


#endif /* SPARKY_HPP_ */
