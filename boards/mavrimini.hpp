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
 * \file    mavrimini.hpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Autopilot board based on STM32
 *
 ******************************************************************************/


#ifndef MAVRIMINI_HPP_
#define MAVRIMINI_HPP_

#include "hal/stm32/gpio_stm32.hpp"
#include "hal/stm32/i2c_stm32.hpp"
#include "hal/stm32/pwm_stm32.hpp"
#include "hal/stm32/serial_stm32.hpp"

#include "drivers/airspeed_analog.hpp"
#include "drivers/battery.hpp"
#include "drivers/servo.hpp"
#include "drivers/sonar_i2cxl.hpp"
#include "drivers/spektrum_satellite.hpp"
#include "drivers/state_display_mavrimini.hpp"

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
    gpio_stm32_conf_t       green_led_gpio_config;
    gpio_stm32_conf_t       red_led_gpio_config;
    serial_stm32_conf_t     serial_1_config;
    serial_stm32_conf_t     serial_2_config;
    i2c_stm32_conf_t        i2c_1_config;
    i2c_stm32_conf_t        i2c_2_config;
    imu_conf_t              imu_config;
    Pwm_stm32::config_t     pwm_config[8];
    servo_conf_t            servo_config[8];
} mavrimini_conf_t;


/**
 * \brief   Default configuration for the board
 *
 * \return  Config structure
 */
static inline mavrimini_conf_t mavrimini_default_config();


/**
 * \brief  Boardsupport for the MegaFly board (rev4)
 *
 */
class Mavrimini
{
public:
    /**
     * \brief           Constructor
     *
     * \param   config  Board configuration
     */
    Mavrimini(mavrimini_conf_t config = mavrimini_default_config());


    /**
     * \brief   Hardware initialisation

     * \return  Success
     */
    bool init(void);

    /**
     * Public Members
     */
    Gpio_stm32              dsm_receiver_gpio;
    Gpio_stm32              dsm_power_gpio;
    Gpio_stm32              green_led_gpio;
    Gpio_stm32              red_led_gpio;
    Led_gpio                green_led;
    Led_gpio                red_led;
    State_display_mavrimini state_display_mavrimini_;
    File_dummy              file_flash;
    Serial_stm32            serial_1;
    Serial_stm32            serial_2;
    I2c_stm32               i2c_1;
    I2c_stm32               i2c_2;
    Spektrum_satellite      spektrum_satellite;
    Sonar_i2cxl             sonar_i2cxl;
    Adc_dummy               adc_battery;
    Battery                 battery;
    Adc_dummy               adc_airspeed;
    Airspeed_analog         airspeed_analog;
    Pwm_stm32               pwm_0;
    Pwm_stm32               pwm_1;
    Pwm_stm32               pwm_2;
    Pwm_stm32               pwm_3;
    Pwm_stm32               pwm_4;
    Pwm_stm32               pwm_5;
    Pwm_dummy               pwm_6;
    Pwm_dummy               pwm_7;
    Servo                   servo_0;
    Servo                   servo_1;
    Servo                   servo_2;
    Servo                   servo_3;
    Servo                   servo_4;
    Servo                   servo_5;
    Servo                   servo_6;
    Servo                   servo_7;
    Dynamic_model_quad_diag sim_model;
    Simulation              sim;
    Imu                     imu;

private:
    byte_stream_t   dbg_stream_;  ///< Temporary member to make print_util work TODO: remove
};


/**
 * \brief   Default configuration for the board
 *
 * \return  Config structure
 */
static inline mavrimini_conf_t mavrimini_default_config()
{
    mavrimini_conf_t conf = {};

    // -------------------------------------------------------------------------
    // GPIO config
    // -------------------------------------------------------------------------

    // GPIO dsm power pin configuration
    conf.dsm_power_gpio_config.port       = GPIO_STM32_PORT_A;
    conf.dsm_power_gpio_config.pin        = GPIO_STM32_PIN_4;
    conf.dsm_power_gpio_config.dir        = GPIO_OUTPUT;
    conf.dsm_power_gpio_config.pull       = GPIO_PULL_UPDOWN_NONE; //physical pull up

    // GPIO dsm receiver pin configuration
    conf.dsm_receiver_gpio_config.port       = GPIO_STM32_PORT_A;
    conf.dsm_receiver_gpio_config.pin        = GPIO_STM32_PIN_3;
    conf.dsm_receiver_gpio_config.dir        = GPIO_INPUT;
    conf.dsm_receiver_gpio_config.pull       = GPIO_PULL_UPDOWN_NONE;

    // Green led
    conf.green_led_gpio_config.port     = GPIO_STM32_PORT_C;
    conf.green_led_gpio_config.pin      = GPIO_STM32_PIN_15;
    conf.green_led_gpio_config.dir      = GPIO_OUTPUT;
    conf.green_led_gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;

    // Red led
    conf.red_led_gpio_config.port   = GPIO_STM32_PORT_C;
    conf.red_led_gpio_config.pin    = GPIO_STM32_PIN_14;
    conf.red_led_gpio_config.dir    = GPIO_OUTPUT;
    conf.red_led_gpio_config.pull   = GPIO_PULL_UPDOWN_NONE;


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

    // -------------------------------------------------------------------------
    // I2C config
    // -------------------------------------------------------------------------
    conf.i2c_1_config                       = i2c_stm32_default_config();
    conf.i2c_1_config.i2c_device_config     = STM32_I2C1;
    conf.i2c_1_config.rcc_i2c_config        = RCC_I2C1;
    conf.i2c_1_config.rcc_sda_port_config   = RCC_GPIOB;
    conf.i2c_1_config.sda_config.port       = GPIO_STM32_PORT_B;
    conf.i2c_1_config.sda_config.pin        = GPIO_STM32_PIN_7;
    conf.i2c_1_config.sda_config.alt_fct    = GPIO_STM32_AF_4;
    conf.i2c_1_config.rcc_clk_port_config   = RCC_GPIOB;
    conf.i2c_1_config.clk_config.port       = GPIO_STM32_PORT_B;
    conf.i2c_1_config.clk_config.pin        = GPIO_STM32_PIN_6;
    conf.i2c_1_config.clk_config.alt_fct    = GPIO_STM32_AF_4;
    conf.i2c_1_config.clk_speed             = 400000;
    conf.i2c_1_config.tenbit_config         = false; // 10 bits address is not supported

    // -------------------------------------------------------------------------
    // I2C config
    // -------------------------------------------------------------------------
    conf.i2c_2_config                       = i2c_stm32_default_config();
    conf.i2c_2_config.i2c_device_config     = STM32_I2C2;
    conf.i2c_2_config.rcc_i2c_config        = RCC_I2C2;
    conf.i2c_2_config.rcc_sda_port_config   = RCC_GPIOB;
    conf.i2c_2_config.sda_config.port       = GPIO_STM32_PORT_B;
    conf.i2c_2_config.sda_config.pin        = GPIO_STM32_PIN_11;
    conf.i2c_2_config.sda_config.alt_fct    = GPIO_STM32_AF_4;
    conf.i2c_2_config.rcc_clk_port_config   = RCC_GPIOB;
    conf.i2c_2_config.clk_config.port       = GPIO_STM32_PORT_B;
    conf.i2c_2_config.clk_config.pin        = GPIO_STM32_PIN_10;
    conf.i2c_2_config.clk_config.alt_fct    = GPIO_STM32_AF_4;
    conf.i2c_2_config.clk_speed             = 100000;
    conf.i2c_2_config.tenbit_config         = false; // 10 bits address is not supported

    // -------------------------------------------------------------------------
    // PWM config
    // -------------------------------------------------------------------------
    conf.pwm_config[0].gpio_config.port     = GPIO_STM32_PORT_B;
    conf.pwm_config[0].gpio_config.pin      = GPIO_STM32_PIN_3;
    conf.pwm_config[0].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[0].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[0].gpio_config.alt_fct  = GPIO_STM32_AF_1;
    conf.pwm_config[0].timer                = TIM2;
    conf.pwm_config[0].rcc_timer            = RCC_TIM2;
    conf.pwm_config[0].channel              = Pwm_stm32::PWM_STM32_CHANNEL_2;
    conf.pwm_config[0].prescaler            = 84; //since APB1 clock is main_clk/2
    conf.pwm_config[0].period               = 20000; //50Hz
    conf.pwm_config[0].pulse_us             = 5000;

    conf.pwm_config[1].gpio_config.port     = GPIO_STM32_PORT_B;
    conf.pwm_config[1].gpio_config.pin      = GPIO_STM32_PIN_4;
    conf.pwm_config[1].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[1].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[1].gpio_config.alt_fct  = GPIO_STM32_AF_2;
    conf.pwm_config[1].timer                = TIM3;
    conf.pwm_config[1].rcc_timer            = RCC_TIM3;
    conf.pwm_config[1].channel              = Pwm_stm32::PWM_STM32_CHANNEL_1;
    conf.pwm_config[1].prescaler            = 84;
    conf.pwm_config[1].period               = 20000; //50Hz
    conf.pwm_config[1].pulse_us             = 5000;

    conf.pwm_config[2].gpio_config.port     = GPIO_STM32_PORT_B;
    conf.pwm_config[2].gpio_config.pin      = GPIO_STM32_PIN_5;
    conf.pwm_config[2].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[2].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[2].gpio_config.alt_fct  = GPIO_STM32_AF_2;
    conf.pwm_config[2].timer                = TIM3;
    conf.pwm_config[2].rcc_timer            = RCC_TIM3;
    conf.pwm_config[2].channel              = Pwm_stm32::PWM_STM32_CHANNEL_2;
    conf.pwm_config[2].prescaler            = 84;
    conf.pwm_config[2].period               = 20000; //50Hz
    conf.pwm_config[2].pulse_us             = 5000;

    conf.pwm_config[3].gpio_config.port     = GPIO_STM32_PORT_B;
    conf.pwm_config[3].gpio_config.pin      = GPIO_STM32_PIN_6;
    conf.pwm_config[3].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[3].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[3].gpio_config.alt_fct  = GPIO_STM32_AF_2;
    conf.pwm_config[3].timer                = TIM4;
    conf.pwm_config[3].rcc_timer            = RCC_TIM4;
    conf.pwm_config[3].channel              = Pwm_stm32::PWM_STM32_CHANNEL_1;
    conf.pwm_config[3].prescaler            = 84;
    conf.pwm_config[3].period               = 20000; //50Hz
    conf.pwm_config[3].pulse_us             = 5000;

    conf.pwm_config[4].gpio_config.port     = GPIO_STM32_PORT_B;
    conf.pwm_config[4].gpio_config.pin      = GPIO_STM32_PIN_7;
    conf.pwm_config[4].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[4].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[4].gpio_config.alt_fct  = GPIO_STM32_AF_2;
    conf.pwm_config[4].timer                = TIM4;
    conf.pwm_config[4].rcc_timer            = RCC_TIM4;
    conf.pwm_config[4].channel              = Pwm_stm32::PWM_STM32_CHANNEL_2;
    conf.pwm_config[4].prescaler            = 84;
    conf.pwm_config[4].period               = 20000; //50Hz
    conf.pwm_config[4].pulse_us             = 5000;

    conf.pwm_config[5].gpio_config.port     = GPIO_STM32_PORT_B;
    conf.pwm_config[5].gpio_config.pin      = GPIO_STM32_PIN_8;
    conf.pwm_config[5].gpio_config.dir      = GPIO_OUTPUT;
    conf.pwm_config[5].gpio_config.pull     = GPIO_PULL_UPDOWN_NONE;
    conf.pwm_config[5].gpio_config.alt_fct  = GPIO_STM32_AF_2;
    conf.pwm_config[5].timer                = TIM4;
    conf.pwm_config[5].rcc_timer            = RCC_TIM4;
    conf.pwm_config[5].channel              = Pwm_stm32::PWM_STM32_CHANNEL_3;
    conf.pwm_config[5].prescaler            = 84;
    conf.pwm_config[5].period               = 20000; //50Hz
    conf.pwm_config[5].pulse_us             = 5000;

    // -------------------------------------------------------------------------
    // Servo config
    // -------------------------------------------------------------------------

    // Warning: servo 0,1,3,5 are configured for a dc brush motor
    // ,servo 2,4 are thus free but servo 1 and 2 are sharing the
    // same clock and so are servo 3,4,5. Servos sharing the same
    // clock cannot have different period therfore all the servos
    // must have the same configuration for now.
    conf.servo_config[0] = servo_default_config_brush_motor();
    conf.servo_config[1] = servo_default_config_brush_motor();
    conf.servo_config[2] = servo_default_config_brush_motor();
    conf.servo_config[3] = servo_default_config_brush_motor();
    conf.servo_config[4] = servo_default_config_brush_motor();
    conf.servo_config[5] = servo_default_config_brush_motor();
    conf.servo_config[6] = servo_default_config_brush_motor();
    conf.servo_config[7] = servo_default_config_brush_motor();

    return conf;
}


#endif /* MAVRIMINI_HPP_ */
