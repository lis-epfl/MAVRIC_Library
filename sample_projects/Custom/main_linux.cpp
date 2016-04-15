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
#include "drivers/flow_px4.hpp"
#include "simulation/flow_sim.hpp"
#include "util/raytracing.hpp"

extern "C"
{
#include "util/print_util.h"
}


#include <iostream>

int main(int argc, char** argv)
{
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

    // -------------------------------------------------------------------------
    // Board Initialisation
    // -------------------------------------------------------------------------
    bool init_success = true;

    // Board initialisation
    init_success &= board.init();
    board.sim.update();

    // -------------------------------------------------------------------------
    // Additionnal sensors
    // -------------------------------------------------------------------------
    File_linux file_log;
    File_linux file_stat;

    // Objects for optic flow
    raytracing::World world;

    uint32_t cyl_row = 4;
    raytracing::Cylinder cylinders[cyl_row*cyl_row];
    for (uint32_t i = 0; i < cyl_row; i++)
    {
        for (uint32_t j = 0; j < cyl_row; j++)
        {
            cylinders[i*cyl_row+j].set_center(Vector3f{ -10.0f + i * 20.0f / cyl_row,
                                                        -10.0f + j * 20.0f / cyl_row,
                                                        0.0f});
            cylinders[i*cyl_row+j].set_axis(Vector3f{ 0.0f, 0.0f, 1.0f});
            cylinders[i*cyl_row+j].set_radius(0.5f);

            world.add_object(&cylinders[i*cyl_row+j]);
        }
    }
    // raytracing::Cylinder cyl(Vector3f{ 0.5f, 1.5f, 0.0f}, Vector3f{ 0.0f, 0.0f, 1.0f}, 1.0f);
    // world.add_object(&cyl);

    raytracing::Plane p0(Vector3f{-20.0f, 0.0f, 0.0f}, Vector3f{1.0f, 0.0f, 0.0f});
    raytracing::Plane p1(Vector3f{20.0f, 0.0f, 0.0f}, Vector3f{-1.0f, 0.0f, 0.0f});
    raytracing::Plane p2(Vector3f{0.0f, -20.0f, 0.0f}, Vector3f{0.0f, 1.0f, 0.0f});
    raytracing::Plane p3(Vector3f{0.0f, 20.0f, 0.0f}, Vector3f{0.0f, -1.0f, 0.0f});
    world.add_object(&p0);
    world.add_object(&p1);
    world.add_object(&p2);
    world.add_object(&p3);

    // Simulated Flow cameras
    Flow_sim flow_left(board.dynamic_model, world, -PI);
    Flow_sim flow_right(board.dynamic_model, world, 0.0f);

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
                                   flow_left,
                                   flow_right);


    // -------------------------------------------------------------------------
    // Initialisation
    // -------------------------------------------------------------------------
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

        // recenter simulation
        local_position_t& pos = board.dynamic_model.position_lf();

        float max_dist = 20.0f;
        if ((pos.pos[0] > max_dist) || (pos.pos[1] > max_dist) || (pos.pos[2] > max_dist) || (pos.pos[0] < -max_dist) || (pos.pos[1] < -max_dist) || (pos.pos[2] < -max_dist))
        {
            pos.pos[0] = 0.0f;
            pos.pos[1] = 0.0f;
            pos.pos[2] = 0.0f;
        }
    }

    return 0;
}
