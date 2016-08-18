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


#include "boards/sparky_chibi/sparky_chibi.hpp"
#include "util/print_util.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "hal.h"
}


// static Serial* p_dbg_serial;
// uint8_t serial2stream(stream_data_t data, uint8_t byte)
// {
//     p_dbg_serial->write(&byte);
//     return 0;
// }


// static void clock_setup(void)
// {
//     // Set STM32 to 168 MHz
//     rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
//
//     // Enable GPIO clock
//     rcc_periph_clock_enable(RCC_GPIOA);
//     rcc_periph_clock_enable(RCC_GPIOB);
//     rcc_periph_clock_enable(RCC_GPIOC);
//     rcc_periph_clock_enable(RCC_GPIOD);
// }


Sparky_chibi::Sparky_chibi(conf_t config):
    gpio_led_err_(config.gpio_led_err),
    gpio_led_stat_(config.gpio_led_stat),
    gpio_led_rf_(config.gpio_led_rf),
    led_err_(gpio_led_err_, false),
    led_stat_(gpio_led_stat_, false),
    led_rf_(gpio_led_rf_, false),
    state_display_(led_err_, led_stat_),
    i2c1_(config.i2c1),
    barometer_(i2c1_, config.barometer)
    // pwm_0_(config.pwm_config[0]),
    // pwm_1_(config.pwm_config[1]),
    // pwm_2_(config.pwm_config[2]),
    // pwm_3_(config.pwm_config[3]),
    // pwm_4_(config.pwm_config[4]),
    // pwm_5_(config.pwm_config[5]),
    // serial_(config.serial_usb_config),
    // servo_0_(pwm_0_, config.servo_config[0]),
    // servo_1_(pwm_1_, config.servo_config[1]),
    // servo_2_(pwm_2_, config.servo_config[2]),
    // servo_3_(pwm_3_, config.servo_config[3]),
    // servo_4_(pwm_4_, config.servo_config[4]),
    // servo_5_(pwm_5_, config.servo_config[5]),
    // servo_6_(pwm_6_, config.servo_config[6]),
    // servo_7_(pwm_7_, config.servo_config[7]),
    // spi_1_(config.spi_config[0]),
    // spi_3_(config.spi_config[2]),
    // i2c_2_(config.i2c_config[1]),
    // state_display_sparky_v2_(led_stat_, led_err_),
{}


bool Sparky_chibi::init(void)
{
    bool init_success = true;
    bool ret;

    // -------------------------------------------------------------------------
    // Init HAL
    // -------------------------------------------------------------------------
    // HAL initialization, this also initializes the configured device drivers and performs
    // the board-specific initializations.
    halInit();

    // Enabling interrupts, initialization done.
    osalSysEnable();

    // Init time keeper
    time_keeper_init();

    // -------------------------------------------------------------------------
    // Init LEDs
    // -------------------------------------------------------------------------
    ret = gpio_led_err_.init();
    ret = gpio_led_stat_.init();
    ret = gpio_led_rf_.init();
    init_success &= ret;
    time_keeper_delay_ms(10);

    // -------------------------------------------------------------------------
    // Init I2Cs
    // -------------------------------------------------------------------------
    ret = i2c1_.init();
    init_success &= ret;
    time_keeper_delay_ms(10);

    // -------------------------------------------------------------------------
    // Init PWMs
    // -------------------------------------------------------------------------
    ret = pwm1_.init();
    init_success &= ret;
    time_keeper_delay_ms(10);


    // -------------------------------------------------------------------------
    // Init barometer
    // -------------------------------------------------------------------------
    ret = barometer_.init();
    init_success &= ret;

    // // -------------------------------------------------------------------------
    // // Init SERIAL
    // // -------------------------------------------------------------------------
    // ret &= serial_.init();
    // time_keeper_delay_ms(500);  // This delay is required to let the host computer initialize the usb serial interface
    // init_success &= ret;

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
    // Init Servos
    // -------------------------------------------------------------------------
// #if CALIBRATE_ESC == 0
//     // Do not calibrate esc
//     ret = pwm_0_.init();
//     // print_util_dbg_init_msg("[PWM0]", ret);
//     init_success &= ret;
//     servo_0_.failsafe();
//     // p_dbg_serial->flush();
//     ret = pwm_1_.init();
//     // print_util_dbg_init_msg("[PWM1]", ret);
//     init_success &= ret;
//     servo_1_.failsafe();
//     // p_dbg_serial->flush();
//     ret = pwm_2_.init();
//     // print_util_dbg_init_msg("[PWM2]", ret);
//     init_success &= ret;
//     servo_2_.failsafe();
//     // p_dbg_serial->flush();
//     ret = pwm_3_.init();
//     // print_util_dbg_init_msg("[PWM3]", ret);
//     init_success &= ret;
//     servo_3_.failsafe();
//     // p_dbg_serial->flush();
//     ret = pwm_4_.init();
//     // print_util_dbg_init_msg("[PWM4]", ret);
//     init_success &= ret;
//     servo_4_.failsafe();
//     // p_dbg_serial->flush();
//     ret = pwm_5_.init();
//     // print_util_dbg_init_msg("[PWM5]", ret);
//     init_success &= ret;
//     servo_5_.failsafe();
//     // p_dbg_serial->flush();
//     init_success &= ret;
//     ret = pwm_6_.init();
//     // print_util_dbg_init_msg("[PWM6]", ret);
//     init_success &= ret;
//     servo_6_.failsafe();
//     // p_dbg_serial->flush();
//     ret = pwm_7_.init();
//     // print_util_dbg_init_msg("[PWM7]", ret);
//     init_success &= ret;
//     servo_7_.failsafe();
//     // p_dbg_serial->flush();
// #elif CALIBRATE_ESC == 1 // Calibrate the esc
//
//     ret = pwm_0_.init();
//     // print_util_dbg_init_msg("[PWM0]", ret);
//     init_success &= ret;
//     ret = pwm_1_.init();
//     // print_util_dbg_init_msg("[PWM1]", ret);
//     init_success &= ret;
//     ret = pwm_2_.init();
//     // print_util_dbg_init_msg("[PWM2]", ret);
//     init_success &= ret;
//     ret = pwm_3_.init();
//     // print_util_dbg_init_msg("[PWM3]", ret);
//     init_success &= ret;
//     ret = pwm_4_.init();
//     // print_util_dbg_init_msg("[PWM4]", ret);
//     init_success &= ret;
//     ret = pwm_5_.init();
//     // print_util_dbg_init_msg("[PWM5]", ret);
//     init_success &= ret;
//     ret = pwm_6_.init();
//     // print_util_dbg_init_msg("[PWM6]", ret);
//     init_success &= ret;
//     ret = pwm_7_.init();
//     // print_util_dbg_init_msg("[PWM7]", ret);
//     init_success &= ret;
//
//     servo_0_.set_servo_max();
//     servo_1_.set_servo_max();
//     servo_2_.set_servo_max();
//     servo_3_.set_servo_max();
//     servo_4_.set_servo_max();
//     servo_5_.set_servo_max();
//     servo_6_.set_servo_max();
//     servo_7_.set_servo_max();
//
//     time_keeper_delay_ms(3000);
//
//     servo_0_.failsafe();
//     servo_1_.failsafe();
//     servo_2_.failsafe();
//     servo_3_.failsafe();
//     servo_4_.failsafe();
//     servo_5_.failsafe();
//     servo_6_.failsafe();
//     servo_7_.failsafe();
//
//     time_keeper_delay_ms(50);
//
// #else
//
//     // print_util_dbg_init_msg("[PWM0]", false);
//     // print_util_dbg_init_msg("[PWM1]", false);
//     // print_util_dbg_init_msg("[PWM2]", false);
//     // print_util_dbg_init_msg("[PWM3]", false);
//     // print_util_dbg_init_msg("[PWM4]", false);
//     // print_util_dbg_init_msg("[PWM5]", false);
//     // print_util_dbg_init_msg("[PWM6]", false);
//     // print_util_dbg_init_msg("[PWM7]", false);
//
// #endif
//
//     // -------------------------------------------------------------------------
//     // Init SPIs
//     // -------------------------------------------------------------------------
//     ret = spi_1_.init();
//     ret = spi_3_.init();
//     init_success &= ret;
//
//     // -------------------------------------------------------------------------
//     // Init I2Cs
//     // -------------------------------------------------------------------------
//     ret = i2c_1_.init();
//     ret = i2c_2_.init();
//     init_success &= ret;
//     time_keeper_delay_ms(500);



    return init_success;
}
