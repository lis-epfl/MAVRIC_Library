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

#include "boards/mavrinux.hpp"
#include "sample_projects/LEQuad/lequad.hpp"

extern "C"
{
#include "util/print_util.h"
}


int main(int argc, char** argv)
{
    uint8_t sysid = 0;
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Get command line parameters
    // -------------------------------------------------------------------------
    // System id
    if (argc > 1)
    {
        sysid = atoi(argv[1]);
    }

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    mavrinux_conf_t board_config = mavrinux_default_config();

    // Set correct sysid for UDP port and flash filename
    board_config.serial_udp_config.local_port   = 14000 + sysid;
    board_config.flash_filename                 = std::string("flash") + std::to_string(sysid) + std::string(".bin");

    // Create board
    Mavrinux board(board_config);

    File_linux file_log;
    File_linux file_stat;

    // Board initialisation
    init_success &= board.init();

    board.sim.update();

    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    // Create MAV using simulated sensors
    LEQuad::conf_t mav_config = LEQuad::default_config(sysid);
    mav_config.manual_control_config.mode_source = Manual_control::MODE_SOURCE_GND_STATION;
    mav_config.manual_control_config.control_source = Manual_control::CONTROL_SOURCE_NONE;
    mav_config.state_config.simulation_mode = true;

    LEQuad mav = LEQuad(board.imu,
                        board.sim.barometer(),
                        board.sim.gps(),
                        board.sim.sonar(),
                        board.mavlink_serial,
                        board.spektrum_satellite,
                        board.state_display_mavrinux_,
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
                        mav_config);

    print_util_dbg_print("[MAIN] OK. Starting up.\r\n");

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    mav.loop();

    return 0;
}
