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
#include "sample_projects/LEQuad/hexhog.hpp"

#include "boards/megafly_rev4/megafly_rev4.hpp"

#include "hal/dummy/file_dummy.hpp"
#include "hal/avr32/serial_usb_avr32.hpp"

//uncomment to go in simulation
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/piezo_speaker.h"
#include "libs/asf/avr32/services/delay/delay.h"

#include "sample_projects/LEQuad/proj_avr32/config/conf_imu.hpp"

#include "hal/common/mavric_endian.h"
}

// #include "hal/common/dbg.hpp"
void px4_update(I2c& i2c);

int main(void)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    megafly_rev4_conf_t board_config    = megafly_rev4_default_config();

    // Rotate board
    imu_conf_t imu_config = board_config.imu_config;
    uint32_t x_axis = 1;
    float x_sign  = -1.0f;
    uint32_t y_axis = 0;
    float y_sign  = 1.0f;
    uint32_t z_axis = 2;
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

    Megafly_rev4 board = Megafly_rev4(board_config);

    // Board initialisation
    init_success &= board.init();

    File_dummy file1;
    File_dummy file2;

    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    // Create MAV using real sensors
    // LEQuad::conf_t mav_config = LEQuad::default_config(MAVLINK_SYS_ID);
    // LEQuad mav = LEQuad(board.imu,
    Hexhog::conf_t mav_config = Hexhog::default_config(MAVLINK_SYS_ID);
    Hexhog mav = Hexhog(board.imu,
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
                        file1,
                        file2,
                        board.i2c1,
                        mav_config );

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
mav.get_scheduler().add_task(100000, (Scheduler_task::task_function_t)&px4_update, (Scheduler_task::task_argument_t)&board.i2c1);

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    mav.loop();

    return 0;
}


#define FLOW_STAT_COMMAND 47
#define PX4_ADDRESS 0x42+3
#define SECTOR_COUNT 6

typedef struct
{
    int16_t maxima[SECTOR_COUNT];
    int16_t max_pos[SECTOR_COUNT];
    int16_t minima[SECTOR_COUNT];
    int16_t min_pos[SECTOR_COUNT];
    int16_t stddev[SECTOR_COUNT];
    int16_t avg[SECTOR_COUNT];
} __attribute__((packed)) flow_stat_frame_t;

#define FLOW_STAT_FRAME_SIZE sizeof(flow_stat_frame_t)

void print_frame(flow_stat_frame_t& f)
{
    print_util_dbg_print("Maxima:\t");
    for(uint8_t i = 0; i < SECTOR_COUNT; i++)
    {
        print_util_dbg_print_num(f.maxima[i],10);
        print_util_dbg_print("\t");
    }

    print_util_dbg_print("\r\nMaxPos:\t");
    for(uint8_t i = 0; i < SECTOR_COUNT; i++)
    {
        print_util_dbg_print_num(f.max_pos[i],10);
        print_util_dbg_print("\t");
    }

    print_util_dbg_print("\r\nMinima:\t");
    for(uint8_t i = 0; i < SECTOR_COUNT; i++)
    {
        print_util_dbg_print_num(f.minima[i],10);
        print_util_dbg_print("\t");
    }

    print_util_dbg_print("\r\nMinPos:\t");
    for(uint8_t i = 0; i < SECTOR_COUNT; i++)
    {
        print_util_dbg_print_num(f.min_pos[i],10);
        print_util_dbg_print("\t");
    }

    print_util_dbg_print("\r\nSTDDEV:\t");
    for(uint8_t i = 0; i < SECTOR_COUNT; i++)
    {
        print_util_dbg_print_num(f.stddev[i],10);
        print_util_dbg_print("\t");
    }

    print_util_dbg_print("\r\nAvg:\t");
    for(uint8_t i = 0; i < SECTOR_COUNT; i++)
    {
        print_util_dbg_print_num(f.avg[i],10);
        print_util_dbg_print("\t");
    }

    print_util_dbg_print("\r\n");
    print_util_dbg_print("\r\n");

    print_util_get_debug_stream()->flush(print_util_get_debug_stream());
}

void print_buffer(uint8_t* buffer, uint8_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        print_util_dbg_print_num(buffer[i], 10);
        print_util_dbg_print(" ");
    }
    print_util_dbg_print("\r\n");
    print_util_dbg_print("\r\n");

    print_util_get_debug_stream()->flush(print_util_get_debug_stream());
}

void px4_update(I2c& i2c)
{
    uint8_t flow_stat_command = FLOW_STAT_COMMAND;
    flow_stat_frame_t frame;
    i2c.write(&flow_stat_command, 1, PX4_ADDRESS);
    if(i2c.read(reinterpret_cast<uint8_t*>(&frame), FLOW_STAT_FRAME_SIZE, PX4_ADDRESS))
    {
        for (size_t i = 0; i < SECTOR_COUNT; i++)
        {
            frame.maxima[i]     = endian_from_little16(frame.maxima[i]);
            frame.max_pos[i]    = endian_from_little16(frame.max_pos[i]);
            frame.minima[i]     = endian_from_little16(frame.minima[i]);
            frame.min_pos[i]    = endian_from_little16(frame.min_pos[i]);
            frame.avg[i]        = endian_from_little16(frame.avg[i]);
            frame.stddev[i]     = endian_from_little16(frame.stddev[i]);
        }

        print_util_dbg_print("PX4 read\r\n");
        print_frame(frame);
    }

}
