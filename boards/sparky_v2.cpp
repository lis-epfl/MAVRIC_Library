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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


#include "boards/sparky_v2.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
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
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

    // Enable GPIO clock
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
}


Sparky_v2::Sparky_v2(sparky_v2_conf_t config):
    led_err_gpio_(config.led_err_gpio_config),
    led_stat_gpio_(config.led_stat_gpio_config),
    led_rf_gpio_(config.led_rf_gpio_config),
    led_err_(led_err_gpio_),
    led_stat_(led_stat_gpio_),
    led_rf_(led_rf_gpio_),
    state_display_sparky_v2_(led_stat_, led_err_)
    // file_flash(),
    // serial_1(config.serial_1_config),
    // serial_2(config.serial_2_config)
{}


bool Sparky_v2::init(void)
{
    bool init_success = true;
    bool ret = true;

    // -------------------------------------------------------------------------
    // Init clock
    // -------------------------------------------------------------------------
    clock_setup();
    time_keeper_init();

    // -------------------------------------------------------------------------
    // Init LEDs
    // -------------------------------------------------------------------------
    ret &= led_err_gpio_.init();
    ret &= led_stat_gpio_.init();
    ret &= led_rf_gpio_.init();
    led_err_.on();
    led_stat_.on();
    led_rf_.on();

    // -------------------------------------------------------------------------
    // Init SERIAL1
    // -------------------------------------------------------------------------
    // ret = serial_1.init();
    // init_success &= ret;

    // -------------------------------------------------------------------------
    // Init SERIAL2
    // -------------------------------------------------------------------------
    // ret = serial_2.init();
    init_success &= ret;

    // -------------------------------------------------------------------------
    // Init stream for USB debug stream TODO: remove
    // p_dbg_serial        = &serial_1;
    // //p_dbg_serial         = &serial_2;
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

    // print_util_dbg_init_msg("[SERIAL1]", ret);
    // print_util_dbg_init_msg("[SERIAL2]", ret);
    // p_dbg_serial->flush();

    return init_success;
}