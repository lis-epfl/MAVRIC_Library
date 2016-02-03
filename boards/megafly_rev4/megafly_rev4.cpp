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
 * \file    megafly_rev4.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Autopilot board based on AVR32
 *
 ******************************************************************************/

#include "boards/megafly_rev4/megafly_rev4.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"

#include "libs/asf/common/services/clock/sysclk.h"
#include "libs/asf/common/services/sleepmgr/sleepmgr.h"
#include "libs/asf/avr32/services/delay/delay.h"

#include "libs/asf/avr32/drivers/gpio/gpio.h"

#include "hal/analog_monitor.h"
#include "hal/analog_monitor_default_config.h"

#include "hal/piezo_speaker.h"
}


static Serial_usb_avr32* p_uart_usb;
uint8_t serial2stream(stream_data_t data, uint8_t byte)
{
    p_uart_usb->write(&byte);
    return 0;
}

Megafly_rev4::Megafly_rev4(megafly_rev4_conf_t config):
    dsm_receiver_pin(Gpio_avr32(config.dsm_receiver_pin_config)),
    dsm_power_pin(Gpio_avr32(config.dsm_power_pin_config)),
    uart0(Serial_avr32(config.uart0_config)),
    uart1(Serial_avr32(config.uart1_config)),
    uart3(Serial_avr32(config.uart3_config)),
    uart_usb(Serial_usb_avr32(config.uart_usb_config)),
    i2c0(I2c_avr32(config.i2c0_config)),
    i2c1(I2c_avr32(config.i2c1_config)),
    hmc5883l(Hmc5883l(i2c0)),
    lsm330dlc(Lsm330dlc(i2c0)),
    bmp085(Bmp085(i2c0)),
    spektrum_satellite(Spektrum_satellite(uart1, dsm_receiver_pin, dsm_power_pin)),
    red_led(LED_AVR32_ID_2) ,
    green_led(LED_AVR32_ID_1),
    imu(Imu(lsm330dlc, lsm330dlc, hmc5883l, config.imu_config)),
    file_flash(File_flash_avr32("flash.bin")),
    gps_ublox(Gps_ublox(uart3)),
    sonar_i2cxl(Sonar_i2cxl(i2c1)),
    adc_battery(Adc_avr32(analog_monitor, {ANALOG_RAIL_10})),
            battery(Battery(adc_battery)),
            pwm_0(0),
            pwm_1(1),
            pwm_2(2),
            pwm_3(3),
            pwm_4(4),
            pwm_5(5),
            pwm_6(6),
            pwm_7(7),
            servo_0(pwm_0, config.servo_config[0]),
            servo_1(pwm_1, config.servo_config[1]),
            servo_2(pwm_2, config.servo_config[2]),
            servo_3(pwm_3, config.servo_config[3]),
            servo_4(pwm_4, config.servo_config[4]),
            servo_5(pwm_5, config.servo_config[5]),
            servo_6(pwm_6, config.servo_config[6]),
            servo_7(pwm_7, config.servo_config[7])
{}


bool Megafly_rev4::init(void)
{
    bool init_success = true;
    bool ret;


    Disable_global_interrupt();

    // -------------------------------------------------------------------------
    // Legacy boardsupport init (TODO: remove)
    // -------------------------------------------------------------------------
    ret = boardsupport_init();
    init_success &= ret;

    // Switch on the red LED
    red_led.on();

    // -------------------------------------------------------------------------
    // Init UART3
    // -------------------------------------------------------------------------
    ret = uart_usb.init();
    init_success &= ret;

    // -------------------------------------------------------------------------
    // Init stream for USB debug stream TODO: remove
    p_uart_usb = &uart_usb;
    dbg_stream_.get = NULL;
    dbg_stream_.put = &serial2stream;
    dbg_stream_.flush = NULL;
    dbg_stream_.buffer_empty = NULL;
    dbg_stream_.data = NULL;
    print_util_dbg_print_init(&dbg_stream_);
    // -------------------------------------------------------------------------

    print_util_dbg_sep('%');
    time_keeper_delay_ms(50);
    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);
    print_util_dbg_print("[MEGAFLY_REV4] ...\r\n");
    time_keeper_delay_ms(50);
    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);

    // -------------------------------------------------------------------------
    // Init GPIO dsm receiver
    // -------------------------------------------------------------------------
    ret = dsm_receiver_pin.init();
    print_util_dbg_init_msg("[DSM RX PIN]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);

    // -------------------------------------------------------------------------
    // Init GPIO dsm power
    // -------------------------------------------------------------------------
    ret = dsm_power_pin.init();
    print_util_dbg_init_msg("[DSM VCC PIN]", ret);
    init_success &= ret;


    // -------------------------------------------------------------------------
    // Init UART0
    // -------------------------------------------------------------------------
    ret = uart0.init();
    print_util_dbg_init_msg("[UART0]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init UART1
    // -------------------------------------------------------------------------
    ret = uart1.init();
    print_util_dbg_init_msg("[UART1]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init UART3
    // -------------------------------------------------------------------------
    ret = uart3.init();
    print_util_dbg_init_msg("[UART3]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init I2C0
    // -------------------------------------------------------------------------
    ret = i2c0.init();
    print_util_dbg_init_msg("[I2C0]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init I2C1
    // -------------------------------------------------------------------------
    ret = i2c1.init();
    print_util_dbg_init_msg("[I2C1]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init pwm
    // -------------------------------------------------------------------------
    ret = pwm_0.init();
    print_util_dbg_init_msg("[PWM0]", ret);
    init_success &= ret;
    servo_0.failsafe();
    time_keeper_delay_ms(50);
    ret = pwm_1.init();
    print_util_dbg_init_msg("[PWM1]", ret);
    init_success &= ret;
    servo_1.failsafe();
    time_keeper_delay_ms(50);
    ret = pwm_2.init();
    print_util_dbg_init_msg("[PWM2]", ret);
    init_success &= ret;
    servo_2.failsafe();
    time_keeper_delay_ms(50);
    ret = pwm_3.init();
    print_util_dbg_init_msg("[PWM3]", ret);
    init_success &= ret;
    servo_3.failsafe();
    time_keeper_delay_ms(50);
    ret = pwm_4.init();
    print_util_dbg_init_msg("[PWM4]", ret);
    init_success &= ret;
    servo_4.failsafe();
    time_keeper_delay_ms(50);
    ret = pwm_5.init();
    print_util_dbg_init_msg("[PWM5]", ret);
    init_success &= ret;
    servo_5.failsafe();
    time_keeper_delay_ms(50);
    ret = pwm_6.init();
    print_util_dbg_init_msg("[PWM6]", ret);
    init_success &= ret;
    servo_6.failsafe();
    time_keeper_delay_ms(50);
    ret = pwm_7.init();
    print_util_dbg_init_msg("[PWM7]", ret);
    init_success &= ret;
    servo_7.failsafe();
    time_keeper_delay_ms(50);


    Enable_global_interrupt();


    // -------------------------------------------------------------------------
    // Init magnetometer
    // -------------------------------------------------------------------------
    ret = hmc5883l.init();
    print_util_dbg_init_msg("[HMC]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init gyro and accelero
    // -------------------------------------------------------------------------
    ret = lsm330dlc.init();
    print_util_dbg_init_msg("[LSM]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init barometer
    // -------------------------------------------------------------------------
    ret = bmp085.init();
    print_util_dbg_init_msg("[BMP]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init spektrum_satelitte
    // -------------------------------------------------------------------------
    ret = spektrum_satellite.init();
    print_util_dbg_init_msg("[SAT]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init sonar
    // -------------------------------------------------------------------------
    ret = sonar_i2cxl.init();
    print_util_dbg_init_msg("[SONAR]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);
    print_util_dbg_init_msg("[MEGAFLY_REV4]", init_success);
    time_keeper_delay_ms(50);
    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);


    return init_success;
}


bool Megafly_rev4::boardsupport_init(void)
{
    bool init_success = true;

    irq_initialize_vectors();
    cpu_irq_enable();
    Disable_global_interrupt();

    // Initialize the sleep manager
    sleepmgr_init();
    sysclk_init();

    board_init();
    delay_init(sysclk_get_cpu_hz());
    time_keeper_init();

    INTC_init_interrupts();

    // Init analog rails
    analog_monitor_init(&analog_monitor, analog_monitor_default_config());

    // init 6V enable
    gpio_enable_gpio_pin(AVR32_PIN_PA04);
    gpio_set_gpio_pin(AVR32_PIN_PA04);

    // Init piezo
    piezo_speaker_init_binary();

    Enable_global_interrupt();

    return init_success;
}