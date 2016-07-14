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
 * \file    mavrimini.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Autopilot board based on STM32
 *
 ******************************************************************************/

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


#include "boards/mavrimini.hpp"

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
    rcc_clock_setup_hse_3v3(&hse_25mhz_3v3[CLOCK_3V3_168MHZ]);

    // Enable GPIO clock
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
}


Mavrimini::Mavrimini(mavrimini_conf_t config):
    dsm_receiver_gpio(config.dsm_receiver_gpio_config), 
    dsm_power_gpio(config.dsm_power_gpio_config),
    green_led_gpio(config.green_led_gpio_config),
    red_led_gpio(config.red_led_gpio_config),
    green_led(green_led_gpio),
    red_led(red_led_gpio),
    file_flash(),
    serial_1(config.serial_1_config),
    serial_2(config.serial_2_config),
    i2c_1(config.i2c_1_config),
    i2c_2(config.i2c_2_config),
    spektrum_satellite(serial_2, dsm_receiver_gpio, dsm_power_gpio),
    sonar_i2cxl(i2c_2),
    adc_battery(12.3f),
    battery(adc_battery),
    adc_airspeed(12.0f),
    airspeed_analog(adc_airspeed,airspeed_analog_default_config()),
    pwm_0(config.pwm_config[0]),
    pwm_1(config.pwm_config[1]),
    pwm_2(config.pwm_config[2]),
    pwm_3(config.pwm_config[3]),
    pwm_4(config.pwm_config[4]),
    pwm_5(config.pwm_config[5]),
    servo_0(pwm_0, config.servo_config[0]),
    servo_1(pwm_1, config.servo_config[1]),
    servo_2(pwm_2, config.servo_config[2]),
    servo_3(pwm_3, config.servo_config[3]),
    servo_4(pwm_4, config.servo_config[4]),
    servo_5(pwm_5, config.servo_config[5]),
    servo_6(pwm_6, config.servo_config[6]),
    servo_7(pwm_7, config.servo_config[7]),
    sim_model(servo_0, servo_1, servo_2, servo_3),
    sim(sim_model),
    imu(sim.accelerometer(), sim.gyroscope(), sim.magnetometer()),
    state_display_mavrimini_(green_led, red_led)
{}


bool Mavrimini::init(void)
{
    bool init_success = true;
    bool ret;

    // -------------------------------------------------------------------------
    // Init clock
    // -------------------------------------------------------------------------
    clock_setup();
    time_keeper_init();

    // -------------------------------------------------------------------------
    // Init LEDs
    // -------------------------------------------------------------------------
    ret = green_led_gpio.init();
    ret = red_led_gpio.init();
    green_led.on();
    red_led.on();

    // -------------------------------------------------------------------------
    // Init SERIAL1
    // -------------------------------------------------------------------------
    ret = serial_1.init();
    init_success &= ret;

    // -------------------------------------------------------------------------
    // Init SERIAL2
    // -------------------------------------------------------------------------
    ret = serial_2.init();
    init_success &= ret;

    // -------------------------------------------------------------------------
    // Init stream for USB debug stream TODO: remove
    p_dbg_serial        = &serial_1;
    //p_dbg_serial         = &serial_2;
    dbg_stream_.get     = NULL;
    dbg_stream_.put     = &serial2stream;
    dbg_stream_.flush   = NULL;
    dbg_stream_.buffer_empty = NULL;
    dbg_stream_.data    = NULL;
    print_util_dbg_print_init(&dbg_stream_);
    // -------------------------------------------------------------------------


    print_util_dbg_sep('%');
    p_dbg_serial->flush();
    print_util_dbg_sep('-');
    p_dbg_serial->flush();
    print_util_dbg_print("[MAVRIMINI] ...\r\n");
    p_dbg_serial->flush();
    print_util_dbg_sep('-');
    p_dbg_serial->flush();

    print_util_dbg_init_msg("[SERIAL1]", ret);
    print_util_dbg_init_msg("[SERIAL2]", ret);
    p_dbg_serial->flush();


    // -------------------------------------------------------------------------
    // Init I2C_1
    // -------------------------------------------------------------------------
    ret = i2c_1.init();
    print_util_dbg_init_msg("[I2C_1]", ret);
    p_dbg_serial->flush();
    init_success &= ret;

    // -------------------------------------------------------------------------
    // Init I2C_2
    // -------------------------------------------------------------------------
    ret = i2c_2.init();
    print_util_dbg_init_msg("[I2C_2]", ret);
    p_dbg_serial->flush();
    init_success &= ret;

    // -------------------------------------------------------------------------
    // Init Servos
    // -------------------------------------------------------------------------
#if CALIBRATE_ESC == 0
    // Do not calibrate esc
    ret = pwm_0.init();
    print_util_dbg_init_msg("[PWM0]", ret);
    init_success &= ret;
    servo_0.failsafe();
    p_dbg_serial->flush();
    ret = pwm_1.init();
    print_util_dbg_init_msg("[PWM1]", ret);
    init_success &= ret;
    servo_1.failsafe();
    p_dbg_serial->flush();
    ret = pwm_2.init();
    print_util_dbg_init_msg("[PWM2]", ret);
    init_success &= ret;
    servo_2.failsafe();
    ret = pwm_3.init();
    print_util_dbg_init_msg("[PWM3]", ret);
    init_success &= ret;
    servo_3.failsafe();
    p_dbg_serial->flush();
    ret = pwm_4.init();
    print_util_dbg_init_msg("[PWM4]", ret);
    init_success &= ret;
    servo_4.failsafe();
    p_dbg_serial->flush();
    ret = pwm_5.init();
    print_util_dbg_init_msg("[PWM5]", ret);
    init_success &= ret;
    servo_5.failsafe();
    p_dbg_serial->flush();
    ret = pwm_6.init();
    print_util_dbg_init_msg("[PWM6]", ret);
    init_success &= ret;
    servo_6.failsafe();
    p_dbg_serial->flush();
    ret = pwm_7.init();
    print_util_dbg_init_msg("[PWM7]", ret);
    init_success &= ret;
    servo_7.failsafe();
    p_dbg_serial->flush();
#elif CALIBRATE_ESC == 1 // Calibrate the esc

    ret = pwm_0.init();
    print_util_dbg_init_msg("[PWM0]", ret);
    init_success &= ret;
    ret = pwm_1.init();
    print_util_dbg_init_msg("[PWM1]", ret);
    init_success &= ret;
    ret = pwm_2.init();
    print_util_dbg_init_msg("[PWM2]", ret);
    init_success &= ret;
    ret = pwm_3.init();
    print_util_dbg_init_msg("[PWM3]", ret);
    init_success &= ret;
    ret = pwm_4.init();
    print_util_dbg_init_msg("[PWM4]", ret);
    init_success &= ret;
    ret = pwm_5.init();
    print_util_dbg_init_msg("[PWM5]", ret);
    init_success &= ret;
    ret = pwm_6.init();
    print_util_dbg_init_msg("[PWM6]", ret);
    init_success &= ret;
    ret = pwm_7.init();
    print_util_dbg_init_msg("[PWM7]", ret);
    init_success &= ret;

    servo_0.set_servo_max();
    servo_1.set_servo_max();
    servo_2.set_servo_max();
    servo_3.set_servo_max();
    servo_4.set_servo_max();
    servo_5.set_servo_max();
    servo_6.set_servo_max();
    servo_7.set_servo_max();

    time_keeper_delay_ms(3000);

    servo_0.failsafe();
    servo_1.failsafe();
    servo_2.failsafe();
    servo_3.failsafe();
    servo_4.failsafe();
    servo_5.failsafe();
    servo_6.failsafe();
    servo_7.failsafe();

    time_keeper_delay_ms(50);

#else

    print_util_dbg_init_msg("[PWM0]", false);
    print_util_dbg_init_msg("[PWM1]", false);
    print_util_dbg_init_msg("[PWM2]", false);
    print_util_dbg_init_msg("[PWM3]", false);
    print_util_dbg_init_msg("[PWM4]", false);
    print_util_dbg_init_msg("[PWM5]", false);
    print_util_dbg_init_msg("[PWM6]", false);
    print_util_dbg_init_msg("[PWM7]", false);

#endif


    // -------------------------------------------------------------------------
    // Init spektrum_satelitte
    // -------------------------------------------------------------------------
    ret = spektrum_satellite.init();
    print_util_dbg_init_msg("[SAT]", ret);
    init_success &= ret;
    p_dbg_serial->flush();

    time_keeper_delay_ms(50);

    // -------------------------------------------------------------------------
    // Init sonar
    // -------------------------------------------------------------------------
    ret = sonar_i2cxl.init();
    print_util_dbg_init_msg("[SONAR]", ret);
    init_success &= ret;
    p_dbg_serial->flush();
    time_keeper_delay_ms(50);


    print_util_dbg_sep('-');
    p_dbg_serial->flush();
    print_util_dbg_init_msg("[MAVRIMINI]", init_success);
    p_dbg_serial->flush();
    print_util_dbg_sep('-');
    p_dbg_serial->flush();


    return init_success;
}