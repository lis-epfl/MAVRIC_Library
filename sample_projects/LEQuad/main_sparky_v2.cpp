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

extern "C"
{
#include "util/print_util.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
}

int main(int argc, char** argv)
{   

    uint8_t sysid = 0;
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    sparky_v2_conf_t board_config = sparky_v2_default_config();
    Sparky_v2 board(board_config);

    // Board initialisation
    init_success &= board.init();


    // #############################################################################################
    // #############  Mavlink test##################################################################
    // #############################################################################################
    Mavlink_stream mavlink_stream(board.serial_);
    mavlink_message_t msg;

    while (1)
    {
        board.state_display_sparky_v2_.update();
        // Warning: if too short serial does not work
        time_keeper_delay_ms(500);

        
        // Write mavlink message
        mavlink_msg_heartbeat_pack( 11,     // uint8_t system_id,
                                    50,     // uint8_t component_id,
                                    &msg,   // mavlink_message_t* msg,
                                    0,      // uint8_t type,
                                    0,      // uint8_t autopilot,
                                    0,      // uint8_t base_mode,
                                    0,      // uint32_t custom_mode,
                                    0);     //uint8_t system_status)
        mavlink_stream.send(&msg);
        
    }

    return 0;
}