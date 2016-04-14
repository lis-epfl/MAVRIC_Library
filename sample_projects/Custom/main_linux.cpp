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

#include "central_data.hpp"
#include "mavlink_telemetry.hpp"
#include "tasks.hpp"

#include "boards/mavrinux.hpp"

#include "util/raytracing.hpp"

extern "C"
{
#include "util/print_util.h"
}


#include <iostream>

int main(int argc, char** argv)
{

    raytracing::Ray r({0.0f, 0.0f, 0.0f}, {1.0f, 0.05f, 0.07f});
    raytracing::Sphere s({1.0f, 0.0f, 0.0f}, 0.1f);
    raytracing::Cylinder c({1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.1f);
    raytracing::Intersection i;
    bool hit = false;
    hit = c.intersect(r, i);
    if (hit)
    {
        std::cout << "Cylinder" << std::endl;
        std::cout << i.point()[0] << " " << i.point()[1] << " " << i.point()[2] << std::endl;
        std::cout << i.normal()[0] << " " << i.normal()[1] << " " << i.normal()[2] << std::endl;
        std::cout << i.distance() << std::endl;
    }

    hit = s.intersect(r, i);
    if (hit)
    {
        std::cout << "Sphere" << std::endl;
        std::cout << i.point()[0] << " " << i.point()[1] << " " << i.point()[2] << std::endl;
        std::cout << i.normal()[0] << " " << i.normal()[1] << " " << i.normal()[2] << std::endl;
        std::cout << i.distance() << std::endl;
    }




    uint8_t sysid = 0;

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
    board_config.dynamic_model_config.rotor_cd  = 1.0f;
    board_config.dynamic_model_config.vehicle_drag  = 10.0f;
    // Create board
    Mavrinux board(board_config);

    File_linux file_log;
    File_linux file_stat;

    // -------------------------------------------------------------------------
    // Serial for PX4Flow cameras
    // -------------------------------------------------------------------------
    Serial_dummy serial_flow_left;
    Serial_dummy serial_flow_right;

    // -------------------------------------------------------------------------
    // Create central data
    // -------------------------------------------------------------------------
    // Create central data using simulated sensors
    Central_data cd = Central_data(sysid,
                                   board.imu,
                                   board.sim.barometer(),
                                   board.sim.gps(),
                                   board.sim.sonar(),
                                   board.mavlink_serial,
                                   board.spektrum_satellite,
                                   board.led,
                                   board.file_flash,
                                   board.battery,
                                   board.servo_0,
                                   board.servo_1,
                                   board.servo_2,
                                   board.servo_3,
                                   file_log,
                                   file_stat,
                                  //  board.mavlink_serial,
                                  //  board.mavlink_serial);
                                   serial_flow_left,
                                   serial_flow_right);


    // -------------------------------------------------------------------------
    // Initialisation
    // -------------------------------------------------------------------------
    bool init_success = true;

    // Board initialisation
    init_success &= board.init();
    board.sim.update();

    // Init central data
    init_success &= cd.init();

    init_success &= mavlink_telemetry_add_onboard_parameters(&cd.mavlink_communication.onboard_parameters, &cd);

    // Try to read from flash, if unsuccessful, write to flash
    if (onboard_parameters_read_parameters_from_storage(&cd.mavlink_communication.onboard_parameters) == false)
    {
        onboard_parameters_write_parameters_to_storage(&cd.mavlink_communication.onboard_parameters);
        init_success = false;
    }

    // init_success &= cd.data_logging.create_new_log_file("Log_file",
    //                 true,
    //                 &cd.toggle_logging,
    //                 &cd.state,
    //                 cd.mavlink_communication.mavlink_stream.sysid);
    //
    // init_success &= cd.data_logging2.create_new_log_file("Log_stat",
    //                 false,
    //                 &cd.toggle_logging,
    //                 &cd.state,
    //                 cd.mavlink_communication.mavlink_stream.sysid);


    init_success &= mavlink_telemetry_init(&cd);

    cd.state.mav_state = MAV_STATE_STANDBY;

    init_success &= tasks_create_tasks(&cd);

    print_util_dbg_print("[MAIN] OK. Starting up.\r\n");

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    while (1 == 1)
    {
        scheduler_update(&cd.scheduler);
    }

    return 0;
}
