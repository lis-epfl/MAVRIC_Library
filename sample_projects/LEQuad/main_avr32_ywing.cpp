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
 * \file main.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief Main file
 *
 ******************************************************************************/

#include "sample_projects/LEQuad/lequad.hpp"
#include "sample_projects/LEQuad/ywing.hpp"

#include "boards/megafly_rev4/megafly_rev4.hpp"

#include "hal/avr32/file_flash_avr32.hpp"
#include "hal/avr32/serial_usb_avr32.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
#include "hal/piezo_speaker.h"
#include "libs/asf/avr32/services/delay/delay.h"

#include "sample_projects/LEQuad/proj_avr32/config/conf_imu.hpp"
}

// #include "hal/common/dbg.hpp"

int main(void)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    // Get default config
    megafly_rev4_conf_t board_config    = megafly_rev4_default_config();

    imu_conf_t imu_config = board_config.imu_config;

    // Rotate board
    // board_config.imu_config.accelerometer.sign[0] = -1.0f;
    // board_config.imu_config.accelerometer.sign[1] = 1.0f;
    // board_config.imu_config.accelerometer.sign[2] = -1.0f;
    // board_config.imu_config.accelerometer.axis[0] = 1;
    // board_config.imu_config.accelerometer.axis[1] = 2;
    // board_config.imu_config.accelerometer.axis[2] = 0;
    // board_config.imu_config.gyroscope.sign[0] = -1.0f;
    // board_config.imu_config.gyroscope.sign[1] = 1.0f;
    // board_config.imu_config.gyroscope.sign[2] = -1.0f;
    // board_config.imu_config.gyroscope.axis[0] = 1;
    // board_config.imu_config.gyroscope.axis[1] = 2;
    // board_config.imu_config.gyroscope.axis[2] = 0;
    // board_config.imu_config.magnetometer.sign[0] = -1.0f;
    // board_config.imu_config.magnetometer.sign[1] = 1.0f;
    // board_config.imu_config.magnetometer.sign[2] = -1.0f;
    // board_config.imu_config.magnetometer.axis[0] = 1;
    // board_config.imu_config.magnetometer.axis[1] = 2;
    // board_config.imu_config.magnetometer.axis[2] = 0;

    uint32_t x_axis = 2;
    float x_sign  = -1.0f;
    uint32_t y_axis = 0;
    float y_sign  = -1.0f;
    uint32_t z_axis = 1;
    float z_sign  = 1.0f;

    board_config.imu_config.accelerometer.sign[0] = x_sign * imu_config.accelerometer.sign[x_axis];
    board_config.imu_config.accelerometer.sign[1] = y_sign * imu_config.accelerometer.sign[y_axis];
    board_config.imu_config.accelerometer.sign[2] = z_sign * imu_config.accelerometer.sign[z_axis];
    board_config.imu_config.accelerometer.axis[0] = imu_config.accelerometer.axis[x_axis];
    board_config.imu_config.accelerometer.axis[1] = imu_config.accelerometer.axis[y_axis];
    board_config.imu_config.accelerometer.axis[2] = imu_config.accelerometer.axis[z_axis];
    board_config.imu_config.gyroscope.sign[0] = x_sign * imu_config.gyroscope.sign[x_axis];
    board_config.imu_config.gyroscope.sign[1] = y_sign * imu_config.gyroscope.sign[y_axis];
    board_config.imu_config.gyroscope.sign[2] = z_sign * imu_config.gyroscope.sign[z_axis];
    board_config.imu_config.gyroscope.axis[0] = imu_config.gyroscope.axis[x_axis];
    board_config.imu_config.gyroscope.axis[1] = imu_config.gyroscope.axis[y_axis];
    board_config.imu_config.gyroscope.axis[2] = imu_config.gyroscope.axis[z_axis];
    board_config.imu_config.magnetometer.sign[0] = x_sign * imu_config.magnetometer.sign[x_axis];
    board_config.imu_config.magnetometer.sign[1] = y_sign * imu_config.magnetometer.sign[y_axis];
    board_config.imu_config.magnetometer.sign[2] = z_sign * imu_config.magnetometer.sign[z_axis];
    board_config.imu_config.magnetometer.axis[0] = imu_config.magnetometer.axis[x_axis];
    board_config.imu_config.magnetometer.axis[1] = imu_config.magnetometer.axis[y_axis];
    board_config.imu_config.magnetometer.axis[2] = imu_config.magnetometer.axis[z_axis];

    // Change servos mode
    board_config.servo_config[0] = servo_default_config_esc();
    board_config.servo_config[1] = servo_default_config_standard();
    board_config.servo_config[2] = servo_default_config_standard();
    board_config.servo_config[3] = servo_default_config_standard();

    // Change battery config
    board_config.battery_config.type = BATTERY_LIPO_1S;

    // Create board
    Megafly_rev4 board = Megafly_rev4(board_config);

    // Board initialisation
    init_success &= board.init();


    // -------------------------------------------------------------------------
    // Create files
    // -------------------------------------------------------------------------
    fat_fs_mounting_t fat_fs_mounting;
    fat_fs_mounting_init(&fat_fs_mounting);
    File_fat_fs file_log(true, &fat_fs_mounting); // boolean value = debug mode
    File_fat_fs file_stat(true, &fat_fs_mounting); // boolean value = debug mode

    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    Ywing::conf_t mav_config = Ywing::default_config(MAVLINK_SYS_ID);
    Ywing mav = Ywing(board.imu,
                      board.bmp085,
                      board.gps_ublox,
                      board.sonar_i2cxl,      // Warning:
                      board.uart0,
                      board.spektrum_satellite,
                      board.green_led,
                      board.file_flash,
                      board.battery,
                      board.servo_0,
                      board.servo_1,
                      board.servo_2,
                      board.servo_3,
                      board.servo_4,
                      board.servo_5,
                      board.servo_6,
                      board.servo_7,
                      file_log,
                      file_stat,
                      mav_config );

    // -------------------------------------------------------------------------
    // Notify startup
    // -------------------------------------------------------------------------
    if (init_success)
    {
        piezo_speaker_quick_startup();

        // Switch off red LED
        board.red_led.off();
    }
    else
    {
        piezo_speaker_critical_error_melody();
    }

    print_util_dbg_print("[MAIN] OK. Starting up.\r\n");

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    mav.loop();

    return 0;
}
