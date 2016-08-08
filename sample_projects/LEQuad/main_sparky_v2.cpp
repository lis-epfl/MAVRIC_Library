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
 * \file main_sparky_v2.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief Main file
 *
 ******************************************************************************/

#include "boards/sparky_v2.hpp"

#include "sample_projects/LEQuad/lequad.hpp"
#include "hal/common/time_keeper.hpp"

#include "hal/stm32/spi_stm32.hpp"

#include "hal/dummy/serial_dummy.hpp"
#include "hal/dummy/i2c_dummy.hpp"
#include "hal/dummy/file_dummy.hpp"
#include "hal/dummy/adc_dummy.hpp"
#include "hal/dummy/pwm_dummy.hpp"

#include "simulation/dynamic_model_quad_diag.hpp"
#include "simulation/simulation.hpp"

#include "drivers/spektrum_satellite.hpp"
#include "util/string_util.hpp"

extern "C"
{
#include "util/print_util.h"
}

#define MAVLINK_SYS_ID 2

int main(int argc, char** argv)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    sparky_v2_conf_t board_config = sparky_v2_default_config();
    Sparky_v2 board(board_config);

    // Board initialisation
    init_success &= board.init();

    // -------------------------------------------------------------------------
    // Create devices that are not implemented in board yet
    // -------------------------------------------------------------------------
    // Dummy peripherals
    Serial_dummy        serial_dummy;
    I2c_dummy           i2c_dummy;
    File_dummy          file_dummy;
    Gpio_dummy          gpio_dummy;
    Spektrum_satellite  satellite_dummy(serial_dummy, gpio_dummy, gpio_dummy);

    // -------------------------------------------------------------------------
    // Create simulation
    // -------------------------------------------------------------------------
    // Simulated servos
    Pwm_dummy pwm[8];
    Servo sim_servo_0(pwm[0], servo_default_config_esc());
    Servo sim_servo_1(pwm[1], servo_default_config_esc());
    Servo sim_servo_2(pwm[2], servo_default_config_esc());
    Servo sim_servo_3(pwm[3], servo_default_config_esc());
    Servo sim_servo_4(pwm[4], servo_default_config_esc());
    Servo sim_servo_5(pwm[5], servo_default_config_esc());
    Servo sim_servo_6(pwm[6], servo_default_config_esc());
    Servo sim_servo_7(pwm[7], servo_default_config_esc());

    // Simulated dynamic model
    Dynamic_model_quad_diag     sim_model(sim_servo_0, sim_servo_1, sim_servo_2, sim_servo_3);
    Simulation                  sim(sim_model);

    // Simulated battery
    Adc_dummy   sim_adc_battery(11.1f);
    Battery     sim_battery(sim_adc_battery);

    // Simulated IMU
    Imu     sim_imu( sim.accelerometer(),
                     sim.gyroscope(),
                     sim.magnetometer() );

    // set the flag to simulation
    LEQuad::conf_t mav_config = LEQuad::default_config(MAVLINK_SYS_ID);
    LEQuad mav = LEQuad( sim_imu,
                         sim.barometer(),
                         sim.gps(),
                         sim.sonar(),
                         board.serial_,                // mavlink serial
                         satellite_dummy,
                         board.state_display_sparky_v2_,
                        file_dummy,
                         sim_battery,
                         sim_servo_0,
                         sim_servo_1,
                         sim_servo_2,
                         sim_servo_3 ,
                         sim_servo_4,
                         sim_servo_5,
                         sim_servo_6,
                         sim_servo_7 ,
                         file_dummy,
                         file_dummy,
                         mav_config );

    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    // Create MAV using real sensors
    // LEQuad::conf_t mav_config = LEQuad::default_config(MAVLINK_SYS_ID);
    // LEQuad mav = LEQuad(board.imu,
    //                     board.bmp085,
    //                     board.gps_ublox,
    //                     board.sonar_i2cxl,
    //                     board.uart0,
    //                     board.spektrum_satellite,
    //                     board.state_display_megafly_rev4_,
    //                     board.file_flash,
    //                     board.battery,
    //                     board.servo_0,
    //                     board.servo_1,
    //                     board.servo_2,
    //                     board.servo_3,
    //                     board.servo_4,
    //                     board.servo_5,
    //                     board.servo_6,
    //                     board.servo_7,
    //                     file_dummy,
    //                     file_dummy,
    //                     mav_config );



    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    // mav.loop();

    // #############################################################################################
    // #############  SPI test##################################################################
    // #############################################################################################
    // uint8_t reg  = 0x75 | 0x80; //0x75 | 0x80;
    // uint8_t fil  = 0x55;
    // uint8_t an1 =  10;
    // uint8_t an2  = 20;
    // uint8_t b[5] = {0xf5,0x00,0x33,0x44,0x55};
    // uint8_t *bytes = b;
    // uint8_t dis[2] = {0x6A,0x08};

    // board.spi_1_.write(dis,2);

    board.led_err_.off();
    board.led_stat_.off();
    board.led_rf_.off();

    Console<Serial> console(board.serial_);

    while (1)
    {
        //board.state_display_sparky_v2_.update();
        // Warning: if too short serial does not work
        //time_keeper_delay_ms(1000);


        // Write mavlink message
        // mavlink_msg_heartbeat_pack( 11,     // uint8_t system_id,
        //                             50,     // uint8_t component_id,
        //                             &msg,   // mavlink_message_t* msg,
        //                             0,      // uint8_t type,
        //                             0,      // uint8_t autopilot,
        //                             0,      // uint8_t base_mode,
        //                             0,      // uint32_t custom_mode,
        //                             0);     //uint8_t system_status)
        // mavlink_stream.send(&msg);

        // gpio_clear(GPIO_STM32_PORT_C, GPIO_STM32_PIN_4);
        // //board.spi_1_.write(b,1);
        // //board.spi_1_.read(an1,1);
        // spi_send(SPI1, 0xf5);
        // an1 = spi_read(SPI1);
        // spi_send(SPI1, 0);
        // an2 = spi_read(SPI1);
        // gpio_set(GPIO_STM32_PORT_C, GPIO_STM32_PIN_4);

        const uint8_t WRITE_FLAG = 0x7f;
        const uint8_t READ_FLAG  = 0x80;

        // Write reset
        const uint8_t PIOS_MPU60X0_USER_CTRL_REG    = 0x6A;
        const uint8_t PIOS_MPU60X0_PWRMGMT_IMU_RST  = 0X80;
        uint8_t reset_command[] = {WRITE_FLAG & PIOS_MPU60X0_USER_CTRL_REG,
                                   PIOS_MPU60X0_PWRMGMT_IMU_RST};
        board.spi_1_.write(reset_command,
                           sizeof(reset_command));

        // Let the sensor reset
        time_keeper_delay_ms(50);

        // Read WhoAmI
        const uint8_t PIOS_MPU60X0_WHOAMI  = 0x75;
        const uint8_t PIOS_MPU9250_WHOAMI  = 0x71;
        uint8_t whoami_command[] = {READ_FLAG | PIOS_MPU60X0_WHOAMI, 0};
        uint8_t buffer[] = {12, 13};
        board.spi_1_.transfer(  whoami_command,
                                buffer,
                                sizeof(whoami_command));
        // board.spi_1_.read(buffer, 1);

        // board.spi_1_.read(&an2,1);
        // board.spi_3_.write(bytes,2);
        // board.spi_3_.read(&an1);
        // board.spi_3_.read(&an2);

        if (buffer[0] == PIOS_MPU9250_WHOAMI | buffer[1] == PIOS_MPU9250_WHOAMI)
        {
            board.led_stat_.toggle();
        }
        else
        {
            board.led_err_.toggle();
        }
        //gpio_clear(GPIO_STM32_PORT_C, GPIO_STM32_PIN_4);
        time_keeper_delay_ms(500);
        //gpio_set(GPIO_STM32_PORT_C, GPIO_STM32_PIN_4);

        board.serial_.write(buffer, 2);

        // uint8_t ser_buf[str::MAX_DIGITS10_LONG];
        // uint8_t ser_len = 0;
        // str::format_integer(buffer[0], ser_buf, &ser_len);
        // board.serial_.write(ser_buf, ser_len);
        // print_util_dbg_log_value("\r\n wai", buffer[0], 16);
        // console.write(buffer[0]);
        // const char* newline = "\r\n";
        // board.serial_.write((const uint8_t*)newline, sizeof(newline));

    }

    return 0;
}
