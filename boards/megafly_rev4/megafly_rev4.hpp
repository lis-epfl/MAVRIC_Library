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
 * \file    megafly_rev4.hpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Autopilot board based on AVR32
 *
 ******************************************************************************/


#ifndef MEGAFLY_REV4_HPP_
#define MEGAFLY_REV4_HPP_

#include "hal/avr32/gpio_avr32.hpp"
#include "hal/avr32/serial_avr32.hpp"
#include "hal/avr32/serial_usb_avr32.hpp"
#include "hal/avr32/i2c_avr32.hpp"
#include "drivers/hmc5883l.hpp"
#include "drivers/lsm330dlc.hpp"
#include "drivers/bmp085.hpp"
#include "sensing/imu.hpp"
#include "drivers/spektrum_satellite.hpp"
#include "hal/avr32/file_flash_avr32.hpp"
#include "hal/avr32/file_fat_fs.hpp"
#include "drivers/gps_ublox.hpp"
#include "drivers/sonar_i2cxl.hpp"
#include "hal/avr32/adc_avr32.hpp"
#include "drivers/battery.hpp"
#include "hal/avr32/pwm_avr32.hpp"
#include "drivers/servo.hpp"
#include "hal/avr32/led_avr32.hpp"
#include "drivers/airspeed_analog.hpp"

extern "C"
{
#include "hal/avr32/twim_default_config.h"
#include "util/streams.h"
#include "hal/analog_monitor.h"
}


/**
 * \brief   Configuration structure
 */
typedef struct
{
    gpio_avr32_conf_t       dsm_receiver_pin_config;
    gpio_avr32_conf_t       dsm_power_pin_config;
    serial_avr32_conf_t     uart0_config;
    serial_avr32_conf_t     uart1_config;
    serial_avr32_conf_t     uart3_config;
    serial_usb_avr32_conf_t uart_usb_config;
    i2c_avr32_conf_t        i2c0_config;
    i2c_avr32_conf_t        i2c1_config;
    imu_conf_t              imu_config;
    servo_conf_t            servo_config[8];
} megafly_rev4_conf_t;


/**
 * \brief   Default configuration for the board
 *
 * \return  Config structure
 */
static inline megafly_rev4_conf_t megafly_rev4_default_config();


/**
 * \brief  Boardsupport for the MegaFly board (rev4)
 *
 */
class Megafly_rev4
{
public:
    /**
     * \brief           Constructor
     *
     * \param   config  Board configuration
     */
    Megafly_rev4(megafly_rev4_conf_t config = megafly_rev4_default_config());


    /**
     * \brief   Hardware initialisation

     * \return  Success
     */
    bool init(void);

    /**
     * Public Members
     */
    Gpio_avr32          dsm_receiver_pin;
    Gpio_avr32          dsm_power_pin;
    Serial_avr32        uart0;
    Serial_avr32        uart1;
    Serial_avr32        uart3;
    Serial_usb_avr32    uart_usb;
    I2c_avr32           i2c0;
    I2c_avr32           i2c1;
    Hmc5883l            hmc5883l;
    Lsm330dlc           lsm330dlc;
    Bmp085              bmp085;
    Spektrum_satellite  spektrum_satellite;
    Led_avr32           red_led;
    Led_avr32           green_led;
    Imu                 imu;
    File_flash_avr32    file_flash;
    Gps_ublox           gps_ublox;
    Sonar_i2cxl         sonar_i2cxl;
    analog_monitor_t    analog_monitor;
    Adc_avr32           adc_battery;
    Adc_avr32           adc_airspeed;
    Battery             battery;
    Airspeed_analog     airspeed_analog;
    Pwm_avr32           pwm_0;
    Pwm_avr32           pwm_1;
    Pwm_avr32           pwm_2;
    Pwm_avr32           pwm_3;
    Pwm_avr32           pwm_4;
    Pwm_avr32           pwm_5;
    Pwm_avr32           pwm_6;
    Pwm_avr32           pwm_7;
    Servo               servo_0;
    Servo               servo_1;
    Servo               servo_2;
    Servo               servo_3;
    Servo               servo_4;
    Servo               servo_5;
    Servo               servo_6;
    Servo               servo_7;

private:
    byte_stream_t   dbg_stream_;  ///< Temporary member to make print_util work TODO: remove


    /**
     * \brief   Initialize the hardware related elements (communication lines, sensors devices, etc)
     *
     * \detail  Legacy function: TODO move to init() method
     *
     * \param   central_data        The pointer to the structure where all central data is stored
     *
     * \return  The initialization status of each module, succeed == true
     */
    bool boardsupport_init(void);

};


/**
 * \brief   Default configuration for the board
 *
 * \return  Config structure
 */
static inline megafly_rev4_conf_t megafly_rev4_default_config()
{
    megafly_rev4_conf_t conf = {};

    // -------------------------------------------------------------------------
    // GPIO dsm receiver pin configuration
    // -------------------------------------------------------------------------
    conf.dsm_receiver_pin_config     = gpio_avr32_default_config();
    conf.dsm_receiver_pin_config.pin = AVR32_PIN_PD12;


    // -------------------------------------------------------------------------
    // GPIO dsm power pin configuration
    // -------------------------------------------------------------------------
    conf.dsm_power_pin_config     = gpio_avr32_default_config();
    conf.dsm_power_pin_config.pin = AVR32_PIN_PC01;


    // -------------------------------------------------------------------------
    // UART0 configuration
    // -------------------------------------------------------------------------
    conf.uart0_config                       = {};
    conf.uart0_config.serial_device         = AVR32_SERIAL_0;
    conf.uart0_config.mode                  = AVR32_SERIAL_IN_OUT;
    conf.uart0_config.options               = {};
    conf.uart0_config.options.baudrate      = 57600;
    conf.uart0_config.options.charlength    = 8;
    conf.uart0_config.options.paritytype    = USART_NO_PARITY;
    conf.uart0_config.options.stopbits      = USART_1_STOPBIT;
    conf.uart0_config.options.channelmode   = USART_NORMAL_CHMODE;
    conf.uart0_config.rx_pin_map            = {AVR32_USART0_RXD_0_0_PIN, AVR32_USART0_RXD_0_0_FUNCTION};
    conf.uart0_config.tx_pin_map            = {AVR32_USART0_TXD_0_0_PIN, AVR32_USART0_TXD_0_0_FUNCTION};


    // -------------------------------------------------------------------------
    // UART1 configuration
    // -------------------------------------------------------------------------
    conf.uart1_config                       = {};
    conf.uart1_config.serial_device         = AVR32_SERIAL_1;
    conf.uart1_config.mode                  = AVR32_SERIAL_IN_OUT;
    conf.uart1_config.options               = {};
    conf.uart1_config.options.baudrate      = 115200;
    conf.uart1_config.options.charlength    = 8;
    conf.uart1_config.options.paritytype    = USART_NO_PARITY;
    conf.uart1_config.options.stopbits      = USART_1_STOPBIT;
    conf.uart1_config.options.channelmode   = USART_NORMAL_CHMODE;
    conf.uart1_config.rx_pin_map            = {AVR32_USART1_RXD_0_1_PIN, AVR32_USART1_RXD_0_1_FUNCTION};
    conf.uart1_config.tx_pin_map            = {AVR32_USART1_TXD_0_1_PIN, AVR32_USART1_TXD_0_1_FUNCTION};


    // -------------------------------------------------------------------------
    // UART3 configuration
    // -------------------------------------------------------------------------
    conf.uart3_config                       = {};
    conf.uart3_config.serial_device         = AVR32_SERIAL_3;
    conf.uart3_config.mode                  = AVR32_SERIAL_IN_OUT;
    conf.uart3_config.options               = {};
    conf.uart3_config.options.baudrate      = 38400;
    conf.uart3_config.options.charlength    = 8;
    conf.uart3_config.options.paritytype    = USART_NO_PARITY;
    conf.uart3_config.options.stopbits      = USART_1_STOPBIT;
    conf.uart3_config.options.channelmode   = USART_NORMAL_CHMODE;
    conf.uart3_config.rx_pin_map            = {AVR32_USART3_RXD_0_0_PIN, AVR32_USART3_RXD_0_0_FUNCTION};
    conf.uart3_config.tx_pin_map            = {AVR32_USART3_TXD_0_0_PIN, AVR32_USART3_TXD_0_0_FUNCTION};


    // -------------------------------------------------------------------------
    // UART USB configuration
    // -------------------------------------------------------------------------
    conf.uart_usb_config        = {};


    // -------------------------------------------------------------------------
    // I2C0 configuration
    // -------------------------------------------------------------------------
    conf.i2c0_config            = {};
    conf.i2c0_config.i2c_device = AVR32_I2C0;
    conf.i2c0_config.twi_opt    = twim_default_config();
    conf.i2c0_config.tenbit     = false;
    conf.i2c0_config.sda_pin    = AVR32_TWIMS0_TWD_0_0_PIN;
    conf.i2c0_config.clk_pin    = AVR32_TWIMS0_TWCK_0_0_PIN;


    // -------------------------------------------------------------------------
    // I2C1 configuration
    // -------------------------------------------------------------------------
    conf.i2c1_config            = {};
    conf.i2c1_config.i2c_device = AVR32_I2C1;
    conf.i2c1_config.twi_opt    = twim_default_config();
    conf.i2c1_config.tenbit     = false;
    conf.i2c1_config.sda_pin    = AVR32_TWIMS1_TWD_0_0_PIN;
    conf.i2c1_config.clk_pin    = AVR32_TWIMS1_TWCK_0_0_PIN;


    // -------------------------------------------------------------------------
    // Imu config
    // -------------------------------------------------------------------------
    conf.imu_config = imu_default_config();
    // Accelerometer
    // Bias
    conf.imu_config.accelerometer.bias[0] = 0.0f;           ///< Positive or negative
    conf.imu_config.accelerometer.bias[1] = 0.0f;
    conf.imu_config.accelerometer.bias[2] = 0.0f;

    // Scale
    conf.imu_config.accelerometer.scale_factor[0] = 4000.0f;    ///< Should be >0
    conf.imu_config.accelerometer.scale_factor[1] = 4000.0f;
    conf.imu_config.accelerometer.scale_factor[2] = 4000.0f;

    // Axis and sign
    conf.imu_config.accelerometer.sign[0] = +1.0f;  ///< +1 or -1
    conf.imu_config.accelerometer.sign[1] = -1.0f;
    conf.imu_config.accelerometer.sign[2] = -1.0f;
    conf.imu_config.accelerometer.axis[0] = 0;      ///< Should be 0, 1, or 2
    conf.imu_config.accelerometer.axis[1] = 1;
    conf.imu_config.accelerometer.axis[2] = 2;

    // Gyroscope
    // Bias
    conf.imu_config.gyroscope.bias[0] = 0.0f;       ///< Positive or negative
    conf.imu_config.gyroscope.bias[1] = 0.0f;
    conf.imu_config.gyroscope.bias[2] = 0.0f;

    // Scale
    conf.imu_config.gyroscope.scale_factor[0] = 818.5111f;      ///< Should be >0
    conf.imu_config.gyroscope.scale_factor[1] = 818.5111;
    conf.imu_config.gyroscope.scale_factor[2] = 818.5111f;

    // Axis and sign
    conf.imu_config.gyroscope.sign[0] = +1.0f;  ///< +1 or -1
    conf.imu_config.gyroscope.sign[1] = -1.0f;
    conf.imu_config.gyroscope.sign[2] = -1.0f;
    conf.imu_config.gyroscope.axis[0] = 0;      ///< Should be 0, 1, or 2
    conf.imu_config.gyroscope.axis[1] = 1;
    conf.imu_config.gyroscope.axis[2] = 2;

    // Magnetometer
    // Bias
    conf.imu_config.magnetometer.bias[0] = 0.0f;        ///< Positive or negative
    conf.imu_config.magnetometer.bias[1] = 0.0f;
    conf.imu_config.magnetometer.bias[2] = 0.0f;

    // Scale
    conf.imu_config.magnetometer.scale_factor[0] = 600.0f;      ///< Should be >0
    conf.imu_config.magnetometer.scale_factor[1] = 600.0f;
    conf.imu_config.magnetometer.scale_factor[2] = 600.0f;

    // Axis and sign
    conf.imu_config.magnetometer.sign[0] = -1.0f;   ///< +1 or -1
    conf.imu_config.magnetometer.sign[1] = -1.0f;
    conf.imu_config.magnetometer.sign[2] = -1.0f;
    conf.imu_config.magnetometer.axis[0] = 2;       ///< Should be 0, 1, or 2
    conf.imu_config.magnetometer.axis[1] = 0;
    conf.imu_config.magnetometer.axis[2] = 1;



    // -------------------------------------------------------------------------
    // Servo config
    // -------------------------------------------------------------------------
    conf.servo_config[0] = servo_default_config_esc();
    conf.servo_config[1] = servo_default_config_esc();
    conf.servo_config[2] = servo_default_config_esc();
    conf.servo_config[3] = servo_default_config_esc();
    conf.servo_config[4] = servo_default_config_esc();
    conf.servo_config[5] = servo_default_config_esc();
    conf.servo_config[6] = servo_default_config_esc();
    conf.servo_config[7] = servo_default_config_esc();

    return conf;
}


#endif /* MEGAFLY_REV4_HPP_ */
