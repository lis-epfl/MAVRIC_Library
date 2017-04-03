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

#include "drones/lequad.hpp"
#include "sample_projects/LEQuad/proj_avr32/config/conf_imu.hpp"

#include "boards/megafly_rev4/megafly_rev4.hpp"

// #include "hal/dummy/file_dummy.hpp"
#include "hal/avr32/file_flash_avr32.hpp"
#include "hal/avr32/serial_usb_avr32.hpp"

// //uncomment to go in simulation
// #include "simulation/dynamic_model_quad_diag.hpp"
// #include "simulation/simulation.hpp"
// #include "hal/dummy/adc_dummy.hpp"
// #include "hal/dummy/pwm_dummy.hpp"
#include "hal/common/time_keeper.hpp"

#include "util/print_util.hpp"

extern "C"
{
#include "hal/piezo_speaker.h"
#include "libs/asf/avr32/services/delay/delay.h"
}

#include "hal/dummy/serial_dummy.hpp"
#include "sensing/ins_telemetry.hpp"

/* Scheduler header files. */
#include "FreeRTOS.h"
#include "task.h"

#define mainMAVRIC_TASK_PRIORITY ( tskIDLE_PRIORITY + 1 )
static void vMavricTask(void* pvParameters);

#define mainLOGGING_TASK_PRIORITY ( tskIDLE_PRIORITY + 1 )
static void vLoggingTask(void* pvParameters);

int main(void)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    // Board configuration
    megafly_rev4_conf_t board_config    = megafly_rev4_default_config();
    // board_config.uart3_config.serial_device         = AVR32_SERIAL_3;
    // board_config.uart3_config.mode                  = AVR32_SERIAL_IN_OUT;
    // board_config.uart3_config.options.baudrate      = 115200;
    // board_config.uart3_config.options.charlength    = 8;
    // board_config.uart3_config.options.paritytype    = USART_NO_PARITY;
    // board_config.uart3_config.options.stopbits      = USART_1_STOPBIT;
    // board_config.uart3_config.options.channelmode   = USART_NORMAL_CHMODE;
    // board_config.uart3_config.rx_pin_map            = {AVR32_USART3_RXD_0_0_PIN, AVR32_USART3_RXD_0_0_FUNCTION};
    // board_config.uart3_config.tx_pin_map            = {AVR32_USART3_TXD_0_0_PIN, AVR32_USART3_TXD_0_0_FUNCTION};

    // Board initialisation
    Megafly_rev4 board = Megafly_rev4(board_config);
    init_success &= board.init();

    fat_fs_mounting_t fat_fs_mounting;

    fat_fs_mounting_init(&fat_fs_mounting);

    File_fat_fs file_log(true, &fat_fs_mounting); // boolean value = debug mode
    File_fat_fs file_stat(true, &fat_fs_mounting); // boolean value = debug mode

    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    // Create MAV using real sensors
    LEQuad::conf_t mav_config = LEQuad::dronedome_config(MAVLINK_SYS_ID);
    mav_config.mav_config.do_data_logging = false;
    static LEQuad mav = LEQuad(board.imu,
                        board.barometer,
                        board.gps_ublox,
                        board.sonar_i2cxl,
                        board.flow_serial,
                        board.uart0,
                        board.spektrum_satellite,
                        board.state_display_megafly_rev4_,
                        board.file_flash,
                        board.battery,
                        file_log,
                        file_stat,
                        board.servo_0,
                        board.servo_1,
                        board.servo_2,
                        board.servo_3,
                        mav_config );
    // initialize MAV
    init_success &= mav.init();

    // Add logging data
    mav.get_data_logging_continuous().add_field(&mav.sysid_, "SYSID");

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


    // Data logging thread
    if (mav_config.mav_config.do_data_logging == false)
    {
    	xTaskCreate(
            vLoggingTask,
            (const signed portCHAR *)"LoggingTask",
            1024,
            &mav,
            mainLOGGING_TASK_PRIORITY,
            NULL
        );
    }

    // Mavric thread
	xTaskCreate(
        vMavricTask,
        (const signed portCHAR *)"MavricTask",
        1024,
        &mav,
        mainMAVRIC_TASK_PRIORITY,
        NULL
    );


    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    vTaskStartScheduler();
}


static void vMavricTask(void* pvParameters)
{
    MAV& mav = *(MAV*)pvParameters;
    mav.loop();
};


static void vLoggingTask(void* pvParameters)
{
    MAV& mav = *(MAV*)pvParameters;
    Data_logging& data_logging_continuous = mav.get_data_logging_continuous();
    Data_logging& data_logging_stat       = mav.get_data_logging_stat();

    while (1)
    {
        // Wait for the next cycle.
        time_keeper_delay_ms(100);

        // Update data logging
        data_logging_stat.update();
        data_logging_continuous.update();
    }
};
