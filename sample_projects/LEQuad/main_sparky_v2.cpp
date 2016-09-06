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

#include "drivers/rfm22b.hpp"
#include "drivers/spektrum_satellite.hpp"

#include "hal/common/time_keeper.hpp"
#include "hal/dummy/serial_dummy.hpp"
#include "hal/dummy/i2c_dummy.hpp"
#include "hal/dummy/file_dummy.hpp"
#include "hal/dummy/adc_dummy.hpp"
#include "hal/dummy/pwm_dummy.hpp"
#include "hal/stm32/spi_stm32.hpp"

#include "sample_projects/LEQuad/lequad.hpp"

#include "simulation/dynamic_model_quad_diag.hpp"
#include "simulation/simulation.hpp"

#include "util/string_util.hpp"

extern "C"
{
#include "util/print_util.h"
}

#define MAVLINK_SYS_ID 2
#define SEND
#define PKT_SIZE       8

int main(int argc, char** argv)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    sparky_v2_conf_t board_config = sparky_v2_default_config();

    // board_config.imu_config.accelerometer.bias[0] = -0.0327504f;
    // board_config.imu_config.accelerometer.bias[1] = -0.00344232f;
    // board_config.imu_config.accelerometer.bias[2] = +0.00931478f;

    // board_config.imu_config.gyroscope.bias[0] = -0.0135339f;
    // board_config.imu_config.gyroscope.bias[1] = -0.0061096f;
    // board_config.imu_config.gyroscope.bias[2] = -0.00312137f;

    // board_config.imu_config.magnetometer.bias[0] = +0.520405f;
    // board_config.imu_config.magnetometer.bias[1] = -0.55305f;
    // board_config.imu_config.magnetometer.bias[2] = -0.489245f;

    // board_config.imu_config.magnetic_north[0] = +0.268271f;
    // board_config.imu_config.magnetic_north[1] = +0.0f;
    // board_config.imu_config.magnetic_north[2] = +0.485027f;

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

    board.led_err_.off();
    board.led_stat_.off();
    board.led_rf_.off();

    Console<Serial> console(board.serial_);
    Rfm22b rfm22b(board.spi_3_, board.nss_2_gpio_);
    bool bo = rfm22b.init();

    char output[PKT_SIZE] = "Salut";
    char input[PKT_SIZE] = {0};
    int length = PKT_SIZE;
    uint8_t rssi = 0;

    uint8_t interrupt_1 = 0;
    uint8_t interrupt_2 = 0;

    int32_t timeout = 0;
    uint8_t rx_header[4] = {0};
    uint8_t tx_header[4] = {0};

    const char* sep  = " ";
    const char* sep2 = "\t";
    uint64_t delay = 25;

    #if defined (SEND)

    while (1) // Send
    {
        uint8_t device_status = 0;

        // Write data to TX FIFO and send it
        bo &= rfm22b.send((uint8_t*)output, length);

        // Get Device Status
        bo &= rfm22b.read_reg(Rfm22b::DEVICE_STATUS, &device_status);

        // Read Interrupt Status 1
        bo &= rfm22b.read_reg(Rfm22b::INTERRUPT_STAT_1, &interrupt_1);

        // Read Interrupt Status 2
        bo &= rfm22b.read_reg(Rfm22b::INTERRUPT_STAT_2, &interrupt_2);

        // Get Transmit Header
        bo &= rfm22b.get_transmit_header(tx_header);

        uint8_t battery = 0;
        bo &= rfm22b.get_battery_level(&battery);


        console.write(battery);
        time_keeper_delay_ms(delay);
        board.serial_.write((const uint8_t*)sep, sizeof(sep));
        time_keeper_delay_ms(delay);
        board.serial_.write((const uint8_t*)sep2, sizeof(sep));
        time_keeper_delay_ms(delay);

        for (int i = 0; i < length; i++)
        {
            console.write(output[i]);
            time_keeper_delay_ms(delay);
            board.serial_.write((const uint8_t*)sep, sizeof(sep));
            time_keeper_delay_ms(delay);
        }

        board.serial_.write((const uint8_t*)sep2, sizeof(sep));
        time_keeper_delay_ms(delay);

        for (int i = 0; i < 8; i++)
        {
            console.write((device_status >> (7-i)) & 1);
            time_keeper_delay_ms(delay);
            board.serial_.write((const uint8_t*)sep, sizeof(sep));
            time_keeper_delay_ms(delay);
        }

        board.serial_.write((const uint8_t*)sep2, sizeof(sep));
        time_keeper_delay_ms(delay);

        for (int i = 0; i < 8; i++)
        {
            console.write((interrupt_1 >> (7-i)) & 1);
            time_keeper_delay_ms(delay);
            board.serial_.write((const uint8_t*)sep, sizeof(sep));
            time_keeper_delay_ms(delay);
        }

        board.serial_.write((const uint8_t*)sep2, sizeof(sep));
        time_keeper_delay_ms(delay);

        for (int i = 0; i < 8; i++)
        {
            console.write((interrupt_2 >> (7-i)) & 1);
            time_keeper_delay_ms(delay);
            board.serial_.write((const uint8_t*)sep, sizeof(sep));
            time_keeper_delay_ms(delay);
        }

        board.serial_.write((const uint8_t*)sep2, sizeof(sep));
        time_keeper_delay_ms(delay);

        for (int i = 0; i < 4; i++)
        {
            console.write(tx_header[i]);
            time_keeper_delay_ms(delay);
            board.serial_.write((const uint8_t*)sep, sizeof(sep));
            time_keeper_delay_ms(delay);
        }

        const char* newline = "\r\n";
        board.serial_.write((const uint8_t*)newline, sizeof(newline));

        if (bo)
        {
            board.led_stat_.toggle();
        }
        else
        {
            board.led_err_.toggle();
        }

        bo = true;
        length = PKT_SIZE;

        time_keeper_delay_ms(100);
    }

    #else

    while (1) // Receive
    {
        uint8_t device_status = 0;

        // Receive data and read RX FIFO
        bo &= rfm22b.receive((uint8_t*)input, &length);

        // Get Device Status
        bo &= rfm22b.read_reg(Rfm22b::DEVICE_STATUS, &device_status);

        // Read Interrupt Status 1
        bo &= rfm22b.read_reg(Rfm22b::INTERRUPT_STAT_1, &interrupt_1);

        // Read Interrupt Status 2
        bo &= rfm22b.read_reg(Rfm22b::INTERRUPT_STAT_2, &interrupt_2);

        // Read RSSI
        bo &= rfm22b.get_rssi(&rssi);

        // Get Received Header
        bo &= rfm22b.get_received_header(rx_header);

        if ((device_status >> 5) & 1)
        {
            length = 0;
        }

        for (int i = 0; i < length; i++)
        {
            console.write(input[i]);
            time_keeper_delay_ms(delay);
            board.serial_.write((const uint8_t*)sep, sizeof(sep));
            time_keeper_delay_ms(delay);
        }

        board.serial_.write((const uint8_t*)sep2, sizeof(sep));
        time_keeper_delay_ms(delay);

        for (int i = 0; i < 8; i++)
        {
            console.write((device_status >> (7-i)) & 1);
            time_keeper_delay_ms(delay);
            board.serial_.write((const uint8_t*)sep, sizeof(sep));
            time_keeper_delay_ms(delay);
        }

        board.serial_.write((const uint8_t*)sep2, sizeof(sep));
        time_keeper_delay_ms(delay);

        for (int i = 0; i < 8; i++)
        {
            console.write((interrupt_1 >> (7-i)) & 1);
            time_keeper_delay_ms(delay);
            board.serial_.write((const uint8_t*)sep, sizeof(sep));
            time_keeper_delay_ms(delay);
        }

        board.serial_.write((const uint8_t*)sep2, sizeof(sep));
        time_keeper_delay_ms(delay);

        for (int i = 0; i < 8; i++)
        {
            console.write((interrupt_2 >> (7-i)) & 1);
            time_keeper_delay_ms(delay);
            board.serial_.write((const uint8_t*)sep, sizeof(sep));
            time_keeper_delay_ms(delay);
        }

        board.serial_.write((const uint8_t*)sep2, sizeof(sep));
        time_keeper_delay_ms(delay);

        for (int i = 0; i < 4; i++)
        {
            console.write(rx_header[i]);
            time_keeper_delay_ms(delay);
            board.serial_.write((const uint8_t*)sep, sizeof(sep));
            time_keeper_delay_ms(delay);
        }

        board.serial_.write((const uint8_t*)sep2, sizeof(sep));
        time_keeper_delay_ms(delay);

        console.write(rssi);
        time_keeper_delay_ms(delay);
        board.serial_.write((const uint8_t*)sep, sizeof(sep));
        time_keeper_delay_ms(delay);

        const char* newline = "\r\n";
        board.serial_.write((const uint8_t*)newline, sizeof(newline));

        if (bo)
        {
            board.led_stat_.toggle();
        }
        else
        {
            board.led_err_.toggle();
        }

        bo = true;
        length = PKT_SIZE;

        time_keeper_delay_ms(100);
    }
    #endif

    return 0;
}
