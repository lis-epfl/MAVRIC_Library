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

#include "sample_projects/Custom/central_data_custom.hpp"
#include "sample_projects/Custom/mavlink_telemetry.hpp"
#include "sample_projects/Custom/tasks_custom.hpp"

#include "boards/megafly_rev4/megafly_rev4.hpp"

// #include "hal/dummy/file_dummy.hpp"
#include "hal/avr32/file_flash_avr32.hpp"
#include "hal/avr32/serial_usb_avr32.hpp"

#include "hal/dummy/serial_dummy.hpp"
// #include "simulation/dynamic_model_quad_diag.hpp"
// #include "simulation/simulation.hpp"
// #include "hal/dummy/adc_dummy.hpp"
// #include "hal/dummy/pwm_dummy.hpp"

#include "drivers/flow_px4.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
#include "hal/piezo_speaker.h"
#include "libs/asf/avr32/services/delay/delay.h"

#include "config/conf_imu.hpp"
}

// #include "hal/common/dbg.hpp"

int main(void)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    megafly_rev4_conf_t board_config    = megafly_rev4_default_config();
    board_config.imu_config             = imu_config();                         // Load custom imu config (cf conf_imu.h)

    // Battery configuration
    board_config.battery_config.type = BATTERY_LIPO_3S;

    // Modify board configuration for PX4flow
    board_config.uart3_config.serial_device         = AVR32_SERIAL_3;
    board_config.uart3_config.mode                  = AVR32_SERIAL_IN_OUT;
    board_config.uart3_config.options.baudrate      = 115200;
    board_config.uart3_config.options.charlength    = 8;
    board_config.uart3_config.options.paritytype    = USART_NO_PARITY;
    board_config.uart3_config.options.stopbits      = USART_1_STOPBIT;
    board_config.uart3_config.options.channelmode   = USART_NORMAL_CHMODE;
    board_config.uart3_config.rx_pin_map            = {AVR32_USART3_RXD_0_0_PIN, AVR32_USART3_RXD_0_0_FUNCTION};
    board_config.uart3_config.tx_pin_map            = {AVR32_USART3_TXD_0_0_PIN, AVR32_USART3_TXD_0_0_FUNCTION};

    Megafly_rev4 board = Megafly_rev4(board_config);

    // Board initialisation
    init_success &= board.init();

    // Create dummy GPS
    Serial_dummy serial_dummy;
    Gps_ublox gps_dummy(serial_dummy);

    // Files
    fat_fs_mounting_t fat_fs_mounting;
    fat_fs_mounting_init(&fat_fs_mounting);
    File_fat_fs file_log(true, &fat_fs_mounting); // boolean value = debug mode
    File_fat_fs file_stat(true, &fat_fs_mounting); // boolean value = debug mode

    // Flow cameras
    Flow_px4 flow_front(board.uart4);
    Flow_px4 flow_rear(board.uart3);

    // -------------------------------------------------------------------------
    // Create central data
    // -------------------------------------------------------------------------
    // Create central data using real sensors
    Central_data::conf_t cd_config = Central_data::default_config(MAVLINK_SYS_ID);

    Central_data_custom cd = Central_data_custom(board.imu,
                                   board.bmp085,
                                   gps_dummy,
                                   board.sonar_i2cxl,          // Warning:
                                   board.uart0,
                                   board.spektrum_satellite,
                                   board.green_led,
                                   board.file_flash,
                                   board.battery,
                                   board.servo_0,
                                   board.servo_1,
                                   board.servo_2,
                                   board.servo_3,
                                   file_log,
                                   file_stat,
                                   flow_front,   // flow left
                                   flow_rear,
                                   cd_config );  // flow right

    // -------------------------------------------------------------------------
    // Create simulation
    // -------------------------------------------------------------------------
    // // Simulated servos
    // Pwm_dummy pwm[4];
    // Servo sim_servo_0(pwm[0], servo_default_config_esc());
    // Servo sim_servo_1(pwm[1], servo_default_config_esc());
    // Servo sim_servo_2(pwm[2], servo_default_config_esc());
    // Servo sim_servo_3(pwm[3], servo_default_config_esc());

    // // Simulated dynamic model
    // Dynamic_model_quad_diag sim_model    = Dynamic_model_quad_diag(sim_servo_0, sim_servo_1, sim_servo_2, sim_servo_3);
    // Simulation sim                       = Simulation(sim_model);

    // // Simulated battery
    // Adc_dummy    sim_adc_battery = Adc_dummy(11.1f);
    // Battery  sim_battery     = Battery(sim_adc_battery);

    // // Simulated IMU
    // Imu      sim_imu         = Imu(  sim.accelerometer(),
    //                                  sim.gyroscope(),
    //                                  sim.magnetometer() );

    // // set the flag to simulation
    // cd_config.state_config.simulation_mode = HIL_ON;
    // Central_data cd = Central_data( MAVLINK_SYS_ID,
    //                              sim_imu,
    //                              sim.barometer(),
    //                              sim.gps(),
    //                              sim.sonar(),
    //                              board.uart0,                // mavlink serial
    //                              board.spektrum_satellite,
    //                              board.green_led,
    //                              board.file_flash,
    //                              sim_battery,
    //                              sim_servo_0,
    //                              sim_servo_1,
    //                              sim_servo_2,
    //                              sim_servo_3 ,
    //                              file_log,
    //                              file_stat,
    //                              cd_config );

    // Init central data
    init_success &= cd.init();

    Onboard_parameters* onboard_parameters = &cd.mavlink_communication.onboard_parameters();
    init_success &= mavlink_telemetry_add_onboard_parameters(onboard_parameters, &cd);

    print_util_dbg_print("onboard_parameters\r\n");
    delay_ms(150);

    // Try to read from flash, if unsuccessful, write to flash
    if (onboard_parameters->read_parameters_from_storage() == false)
    {
       onboard_parameters->write_parameters_to_storage();
       init_success = false;
    }

    print_util_dbg_print("creating new log files\r\n");
    delay_ms(150);

    init_success &= mavlink_telemetry_init(&cd);

    print_util_dbg_print("mavlink_telemetry_init\r\n");
    delay_ms(150);

    cd.state.mav_state_ = MAV_STATE_STANDBY;

    init_success &= tasks_create_tasks(&cd);

    print_util_dbg_print("tasks_create_tasks\r\n");
    delay_ms(150);

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
    while (1 == 1)
    {
       cd.scheduler.update();
    }

    return 0;
}
