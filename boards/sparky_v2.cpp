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
 * \file    sparky_v2.cpp
 *
 * \author  MAV'RIC Team
 * \author  Jean-Francois Burnier
 *
 * \brief   Autopilot for Sparky V2.0 board based on STM32
 *
 ******************************************************************************/

#include "boards/sparky_v2.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
}


static Serial* p_dbg_serial;
uint8_t serial2stream(stream_data_t data, uint8_t byte)
{
    p_dbg_serial->write(&byte);
    return 0;
}


static void clock_setup(void)
{
    // Set STM32 to 168 MHz
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    // Enable GPIO clock
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
}


Sparky_v2::Sparky_v2(conf_t config):
    led_err_gpio_(config.led_err_gpio_config),
    led_stat_gpio_(config.led_stat_gpio_config),
    led_rf_gpio_(config.led_rf_gpio_config),
    led_err_(led_err_gpio_, false),
    led_stat_(led_stat_gpio_, false),
    led_rf_(led_rf_gpio_, false),
    serial_1_(config.serial_1_config),
    serial_(config.serial_usb_config),
    pwm_({config.pwm_config[0],
          config.pwm_config[1],
          config.pwm_config[2],
          config.pwm_config[3],
          config.pwm_config[4],
          config.pwm_config[5],
          config.pwm_config[6],
          config.pwm_config[7],
          config.pwm_config[8],
          config.pwm_config[9]}),
    servo_({{pwm_[0], config.servo_config[0]},
            {pwm_[1], config.servo_config[1]},
            {pwm_[2], config.servo_config[2]},
            {pwm_[3], config.servo_config[3]},
            {pwm_[4], config.servo_config[4]},
            {pwm_[5], config.servo_config[5]},
            {pwm_[6], config.servo_config[6]},
            {pwm_[7], config.servo_config[7]},
            {pwm_[8], config.servo_config[8]},
            {pwm_[9], config.servo_config[9]}
            }),
    spi_1_(config.spi_config[0]),
    spi_3_(config.spi_config[1]),
    nss_1_gpio_(config.nss_gpio_config[0]),
    nss_2_gpio_(config.nss_gpio_config[1]),
    nss_3_gpio_(config.nss_gpio_config[2]),
    state_display_sparky_v2_(led_stat_, led_err_),
    mpu_9250_(spi_1_, nss_1_gpio_),
    imu_(mpu_9250_, mpu_9250_, mpu_9250_, config.imu_config)
{}


bool Sparky_v2::init(void)
{
    bool init_success = true;
    bool ret;

    // -------------------------------------------------------------------------
    // Init clock
    // -------------------------------------------------------------------------
    clock_setup();
    time_keeper_init();

    // -------------------------------------------------------------------------
    // Init Servos
    // -------------------------------------------------------------------------
#if CALIBRATE_ESC == 0
    for (size_t i = 0; i < PWM_COUNT; i++)
    {
        pwm_[i].init();
        servo_[i].failsafe();
    }
#elif CALIBRATE_ESC == 1 // Calibrate the esc
    for (size_t i = 0; i < PWM_COUNT; i++)
    {
        pwm_[i].init();
        servo_[i].set_servo_max();
    }
    time_keeper_delay_ms(3000);
    for (size_t i = 0; i < PWM_COUNT; i++)
    {
        servo_[i].failsafe();
    }
    time_keeper_delay_ms(50);
#else
#endif

    // -------------------------------------------------------------------------
    // Init LEDs
    // -------------------------------------------------------------------------
    ret = led_err_gpio_.init();
    ret = led_stat_gpio_.init();
    ret = led_rf_gpio_.init();
    led_err_.on();
    led_stat_.on();
    led_rf_.on();

    // -------------------------------------------------------------------------
    // Init SERIAL
    // -------------------------------------------------------------------------
    ret &= serial_.init();
    time_keeper_delay_ms(500);  // This delay is required to let the host computer initialize the usb serial interface
    init_success &= ret;

    // -------------------------------------------------------------------------
    // Init SERIAL
    // -------------------------------------------------------------------------
    ret &= serial_1_.init();
    time_keeper_delay_ms(500);  // This delay is required to let the host computer initialize the usb serial interface
    init_success &= ret;

    // -------------------------------------------------------------------------
    // Init stream for USB debug stream TODO: remove
    // p_dbg_serial        = &serial_;
    // dbg_stream_.get     = NULL;
    // dbg_stream_.put     = &serial2stream;
    // dbg_stream_.flush   = NULL;
    // dbg_stream_.buffer_empty = NULL;
    // dbg_stream_.data    = NULL;
    // print_util_dbg_print_init(&dbg_stream_);
    // -------------------------------------------------------------------------


    // print_util_dbg_sep('%');
    // p_dbg_serial->flush();
    // print_util_dbg_sep('-');
    // p_dbg_serial->flush();
    // print_util_dbg_print("[SPARKY_V2] ...\r\n");
    // p_dbg_serial->flush();
    // print_util_dbg_sep('-');
    // p_dbg_serial->flush();

    // print_util_dbg_init_msg("[SERIAL]", ret);
    // p_dbg_serial->flush();

    // -------------------------------------------------------------------------
    // Init SPIs
    // -------------------------------------------------------------------------
    ret = spi_1_.init();
    ret = spi_3_.init();
    init_success &= ret;

    // -------------------------------------------------------------------------
    // Init IMU
    // -------------------------------------------------------------------------
    ret = mpu_9250_.init();
    init_success &= ret;

    time_keeper_delay_ms(50);


    return init_success;
}
