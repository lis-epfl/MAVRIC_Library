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
 * \file    mavrinux.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Emulated board running on linux
 *
 ******************************************************************************/

#include "boards/mavrinux.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
}


static Serial_linux_io* p_uart_usb;
uint8_t serial2stream(stream_data_t data, uint8_t byte)
{
    p_uart_usb->write(&byte);
    return 0;
}

Mavrinux::Mavrinux(mavrinux_conf_t config):
    servo_0(pwm_0, servo_default_config_esc()),
    servo_1(pwm_1, servo_default_config_esc()),
    servo_2(pwm_2, servo_default_config_esc()),
    servo_3(pwm_3, servo_default_config_esc()),
  	servo_4(pwm_4, servo_default_config_esc()),
  	servo_5(pwm_5, servo_default_config_esc()),
    servo_6(pwm_6, servo_default_config_esc()),
    servo_7(pwm_7, servo_default_config_esc()),
    dynamic_model(servo_0, servo_1, servo_2, servo_3, config.dynamic_model_config),
    sim(dynamic_model),
    imu(sim.accelerometer(), sim.gyroscope(), sim.magnetometer(), config.imu_config),
    adc_battery(12.34f),
    battery(adc_battery),
    spektrum_satellite(dsm_serial, dsm_receiver_pin, dsm_power_pin),
    led(),
    mavlink_serial(config.serial_udp_config),
    file_flash(),
    config_(config)
{}


bool Mavrinux::init(void)
{
    bool init_success = true;
    bool ret;

    // -------------------------------------------------------------------------
    // Init timer
    // -------------------------------------------------------------------------
    time_keeper_init();

    // -------------------------------------------------------------------------
    // Init UART3
    // -------------------------------------------------------------------------
    ret = debug_serial.init();
    init_success &= ret;

    // -------------------------------------------------------------------------
    // Init stream for USB debug stream TODO: remove
    p_uart_usb = &debug_serial;
    dbg_stream_.get = NULL;
    dbg_stream_.put = &serial2stream;
    dbg_stream_.flush = NULL;
    dbg_stream_.buffer_empty = NULL;
    dbg_stream_.data = NULL;
    print_util_dbg_print_init(&dbg_stream_);
    // -------------------------------------------------------------------------


    time_keeper_delay_ms(1000);

    print_util_dbg_sep('%');
    time_keeper_delay_ms(100);
    print_util_dbg_sep('-');
    time_keeper_delay_ms(100);
    print_util_dbg_print("[MAVRINUX] ...\r\n");
    time_keeper_delay_ms(100);
    print_util_dbg_sep('-');
    time_keeper_delay_ms(100);

    // -------------------------------------------------------------------------
    // Init GPIO dsm receiver
    // -------------------------------------------------------------------------
    ret = dsm_receiver_pin.init();
    print_util_dbg_init_msg("[DSM RX PIN]", ret);
    init_success &= ret;
    time_keeper_delay_ms(100);

    // -------------------------------------------------------------------------
    // Init GPIO dsm power
    // -------------------------------------------------------------------------
    ret = dsm_power_pin.init();
    print_util_dbg_init_msg("[DSM VCC PIN]", ret);
    init_success &= ret;


    // -------------------------------------------------------------------------
    // Init UDP serial
    // -------------------------------------------------------------------------
    ret = mavlink_serial.init();
    print_util_dbg_init_msg("[UDP]", ret);
    init_success &= ret;
    time_keeper_delay_ms(100);


    // -------------------------------------------------------------------------
    // Init spektrum_satelitte
    // -------------------------------------------------------------------------
    ret = spektrum_satellite.init();
    print_util_dbg_init_msg("[SAT]", ret);
    init_success &= ret;
    time_keeper_delay_ms(100);

    // -------------------------------------------------------------------------
    // Init file flash
    // -------------------------------------------------------------------------
    file_flash.open(config_.flash_filename.c_str());

    print_util_dbg_sep('-');
    time_keeper_delay_ms(100);
    print_util_dbg_init_msg("[MAVRINUX]", init_success);
    time_keeper_delay_ms(100);
    print_util_dbg_sep('-');
    time_keeper_delay_ms(100);

    return init_success;
}
