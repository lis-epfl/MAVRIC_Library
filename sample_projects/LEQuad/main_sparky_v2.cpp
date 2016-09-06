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

#include "control/manual_control.hpp"

#include "drivers/mpu_9250.hpp"
#include "drivers/spektrum_satellite.hpp"

#include "hal/common/time_keeper.hpp"
#include "hal/dummy/serial_dummy.hpp"
#include "hal/dummy/i2c_dummy.hpp"
#include "hal/dummy/file_dummy.hpp"
#include "hal/dummy/adc_dummy.hpp"
#include "hal/dummy/pwm_dummy.hpp"
#include "hal/stm32/spi_stm32.hpp"

#include "sample_projects/LEQuad/lequad.hpp"
#include "sample_projects/LEQuad/lequad_dronedome.hpp"

#include "simulation/dynamic_model_quad_diag.hpp"
#include "simulation/simulation.hpp"

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

    // board_config.imu_config.accelerometer.bias[0] = -0.0327504f;
    // board_config.imu_config.accelerometer.bias[1] = -0.00344232f;
    // board_config.imu_config.accelerometer.bias[2] = +0.00931478f;

    // board_config.imu_config.gyroscope.bias[0] = -0.0135339f;
    // board_config.imu_config.gyroscope.bias[1] = -0.0061096f;
    // board_config.imu_config.gyroscope.bias[2] = -0.00312137f;

    board_config.imu_config.magnetometer.bias[0] = 0.62785f;
    board_config.imu_config.magnetometer.bias[1] = 0.487535f;
    board_config.imu_config.magnetometer.bias[2] = 0.545637f;

    board_config.imu_config.magnetic_north[0] = +0.689735f;
    board_config.imu_config.magnetic_north[1] = +0.0f;
    board_config.imu_config.magnetic_north[2] = 0.300356f;

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

    // // Simulated battery
    Adc_dummy   sim_adc_battery(11.1f);
    Battery     sim_battery(sim_adc_battery);

    // // Simulated IMU
    // Imu     sim_imu( sim.accelerometer(),
    //                  sim.gyroscope(),
    //                  sim.magnetometer() );

    // // set the flag to simulation
    // LEQuad::conf_t mav_config = LEQuad::default_config(MAVLINK_SYS_ID);
    // LEQuad mav = LEQuad( sim_imu,
    //                      sim.barometer(),
    //                      sim.gps(),
    //                      sim.sonar(),
    //                      board.serial_,                // mavlink serial
    //                      satellite_dummy,
    //                      board.state_display_sparky_v2_,
    //                      file_dummy,
    //                      sim_battery,
    //                      sim_servo_0,
    //                      sim_servo_1,
    //                      sim_servo_2,
    //                      sim_servo_3 ,
    //                      sim_servo_4,
    //                      sim_servo_5,
    //                      sim_servo_6,
    //                      sim_servo_7 ,
    //                      file_dummy,
    //                      file_dummy,
    //                      mav_config );

    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    // Create MAV using real sensors
    LEQuad::conf_t mav_config = LEQuad_dronedome::default_config(MAVLINK_SYS_ID);

    //use joystick by default
    mav_config.manual_control_config.mode_source = Manual_control::MODE_SOURCE_JOYSTICK;
    mav_config.manual_control_config.control_source = Manual_control::CONTROL_SOURCE_JOYSTICK;

    //adapt servo mix gains
    mav_config.stabilisation_copter_config.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].p_gain      = 0.035f;
    mav_config.stabilisation_copter_config.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].integrator.gain      = 0.025f;
    mav_config.stabilisation_copter_config.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].p_gain      = 0.035f;
    mav_config.stabilisation_copter_config.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].integrator.gain      = 0.025f;

    mav_config.stabilisation_copter_config.thrust_hover_point      = 0.0f;

    LEQuad_dronedome mav = LEQuad_dronedome(board.imu_,
                        sim.barometer(),
                        sim.gps(),
                        sim.sonar(),
                        // board.bmp085,
                        // board.gps_ublox,
                        // board.sonar_i2cxl,
                        board.serial_1_,                // mavlink serial
                        // board.serial_,                // mavlink serial
                        satellite_dummy,
                        // board.spektrum_satellite,
                        board.state_display_sparky_v2_,
                        file_dummy,
                        // board.file_flash,
                        sim_battery,
    //                  board.battery,
                        board.servo_0_,
                        board.servo_1_,
                        board.servo_2_,
                        board.servo_3_,
                        board.servo_4_,
                        board.servo_5_,
                        board.servo_6_,
                        board.servo_7_,
                        file_dummy,
                        file_dummy,
                        mav_config );

    mav.init();

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    mav.loop();

    // board.led_err_.off();
    // board.led_stat_.off();
    // board.led_rf_.off();

    // Console<Serial> console(board.serial_);

    // while (1)
    // {

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

    //     time_keeper_delay_ms(500);


    //     const char* sep = "\t";
    //     uint64_t delay = 25;

    //     console.write(valx);
    //     time_keeper_delay_ms(delay);
    //     board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     time_keeper_delay_ms(delay);
    //     console.write(valy);
    //     time_keeper_delay_ms(delay);
    //     board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     time_keeper_delay_ms(delay);
    //     console.write(valz);
    //     time_keeper_delay_ms(delay);

    //     const char* newline = "\r\n";
    //     board.serial_.write((const uint8_t*)newline, sizeof(newline));

    //     if (bo)
    //     {
    //         board.led_stat_.toggle();
    //     }
    //     else
    //     {
    //         board.led_err_.toggle();
    //     }

    // }

    return 0;
}
