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

#include "manual_control/manual_control.hpp"

#include "drivers/gps_ublox.hpp"
#include "drivers/hmc5883l.hpp"
#include "drivers/lsm330dlc.hpp"
#include "drivers/mpu_9250.hpp"
#include "drivers/sonar_i2cxl.hpp"
#include "drivers/spektrum_satellite.hpp"

#include "hal/common/time_keeper.hpp"
#include "hal/dummy/serial_dummy.hpp"
#include "hal/dummy/i2c_dummy.hpp"
#include "hal/dummy/file_dummy.hpp"
#include "hal/dummy/adc_dummy.hpp"
#include "hal/dummy/pwm_dummy.hpp"
#include "hal/stm32/spi_stm32.hpp"

#include "drones/lequad.hpp"

#include "simulation/dynamic_model_quad_diag.hpp"
#include "simulation/simulation.hpp"

#include "util/string_util.hpp"
#include "util/print_util.hpp"


#define MAVLINK_SYS_ID 2

int main(int argc, char** argv)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    Sparky_v2::conf_t board_config = Sparky_v2::default_config();

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
    Adc_dummy           adc_dummy(11.1f);

    // Dummy sensors
    Px4flow_i2c         flow_dummy(i2c_dummy);
    Battery             battery_dummy(adc_dummy);
    Gps_ublox           gps_dummy(serial_dummy);
    Sonar_i2cxl         sonar_dummy(i2c_dummy);
    Barometer_MS5611    barometer_dummy(i2c_dummy);

    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    // Create MAV using real sensors
    //LEQuad::conf_t mav_config = LEQuad::default_config(MAVLINK_SYS_ID);
    LEQuad::conf_t mav_config = LEQuad::dronedome_config(MAVLINK_SYS_ID);
    mav_config.mav_config.ahrs_ekf_config.use_magnetometer = 0;

    //use joystick by default
    mav_config.mav_config.manual_control_config.mode_source = Manual_control::MODE_SOURCE_JOYSTICK;
    mav_config.mav_config.manual_control_config.control_source = Manual_control::CONTROL_SOURCE_JOYSTICK;

    //adapt servo mix gains
    // mav_config.stabilisation_copter_config.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].p_gain      = 0.035f;
    // mav_config.stabilisation_copter_config.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].integrator.gain      = 0.025f;
    // mav_config.stabilisation_copter_config.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].p_gain      = 0.035f;
    // mav_config.stabilisation_copter_config.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].integrator.gain      = 0.025f;
    // mav_config.stabilisation_copter_config.thrust_hover_point      = 0.0f;

    LEQuad mav = LEQuad(board.imu_,
                        barometer_dummy,
                        gps_dummy,
                        sonar_dummy,
                        flow_dummy,
                        board.serial_1_,                // mavlink serial
                        satellite_dummy,
                        board.state_display_sparky_v2_,
                        file_dummy,
                        battery_dummy,
                        file_dummy,
                        file_dummy,
                        board.servo_[0],
                        board.servo_[1],
                        board.servo_[2],
                        board.servo_[3],
                        mav_config );
    mav.init();

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    mav.loop();


    return 0;
}
