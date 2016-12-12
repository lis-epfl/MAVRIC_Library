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
class My_LEQuad: public LEQuad
{
public:
    My_LEQuad( Imu& imu,
            Barometer& barometer,
            Gps& gps,
            Sonar& sonar,
            Px4flow_i2c& flow,
            Serial& serial_mavlink,
            Satellite& satellite,
            State_display& state_display,
            File& file_flash,
            Battery& battery,
            File& file1,
            File& file2,
            Servo& servo_0,
            Servo& servo_1,
            Servo& servo_2,
            Servo& servo_3,
            const conf_t& config = default_config()):
        LEQuad(imu, barometer, gps, sonar, flow, serial_mavlink, satellite, state_display, file_flash, battery, file1, file2, servo_0, servo_1, servo_2, servo_3, config),
        serial_dummy_(),
        gps_dummy_(serial_dummy_),
        ins_no_gps_(state, barometer, sonar, gps_dummy_, flow, ahrs_, config.mav_config.ins_complementary_config)
    {};

    bool init(void)
    {
        bool ret = LEQuad::init();
        ret &= init_ins_no_gps();
        return ret;
    }

    bool main_task(void)
    {
        bool ret = LEQuad::main_task();
        ret &= ins_no_gps_.update();
        return ret;
    }

    bool init_ins_no_gps(void)
    {
        bool ret = true;
        // DOWN telemetry
        ret &= communication.telemetry().add<INS>(MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV, 250000, &ins_telemetry_send_local_position_ned_cov,  &ins_no_gps_);

        // Parameters
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_gps_XY_pos,     "POS2_K_GPS_XY"    );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_gps_Z_pos,      "POS2_K_GPS_Z"     );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_gps_XY_vel,     "POS2_K_GPS_V_XY"  );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_gps_Z_vel,      "POS2_K_GPS_V_Z"   );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_gps_XY_pos_rtk, "POS2_K_RTK_XY"    );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_gps_Z_pos_rtk,  "POS2_K_RTK_Z"     );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_gps_XY_vel_rtk, "POS2_K_RTK_V_XY"  );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_gps_Z_vel_rtk,  "POS2_K_RTK_V_Z"   );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_baro_alt,       "POS2_K_BARO_Z"    );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_baro_vel,       "POS2_K_BARO_V_Z"  );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_sonar_alt,      "POS2_K_SONAR_Z"   );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_sonar_vel,      "POS2_K_SONAR_VZ" );
        ret &= communication.parameters().add(&ins_no_gps_.config_.kp_flow_vel,       "POS2_K_OF_V_XY"   );
        ret &= communication.parameters().add(&ins_no_gps_.config_.use_gps,           "POS2_USE_GPS"     );
        ret &= communication.parameters().add(&ins_no_gps_.config_.use_baro,          "POS2_USE_BARO"    );
        ret &= communication.parameters().add(&ins_no_gps_.config_.use_sonar,         "POS2_USE_SONAR"   );
        ret &= communication.parameters().add(&ins_no_gps_.config_.use_flow,          "POS2_USE_FLOW"    );

        return ret;
    }

protected:
    Serial_dummy        serial_dummy_;
    Gps_ublox           gps_dummy_;
    INS_complementary   ins_no_gps_;
};


int main(void)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    megafly_rev4_conf_t board_config    = megafly_rev4_default_config();
    // board_config.imu_config             = imu_config();                         // Load custom imu config (cf conf_imu.h)
    Megafly_rev4 board = Megafly_rev4(board_config);

    // Board initialisation
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
    // LEQuad mav = LEQuad(board.imu,
    My_LEQuad mav = My_LEQuad(board.imu,
                        board.barometer,
                        board.gps_ublox,
                        board.sonar_i2cxl,
                        board.flow,
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

    // // -------------------------------------------------------------------------
    // // Create simulation
    // // -------------------------------------------------------------------------

    // // // Simulated servos
    // Pwm_dummy pwm[8];
    // Servo sim_servo_0(pwm[0], servo_default_config_esc());
    // Servo sim_servo_1(pwm[1], servo_default_config_esc());
    // Servo sim_servo_2(pwm[2], servo_default_config_esc());
    // Servo sim_servo_3(pwm[3], servo_default_config_esc());
    // Servo sim_servo_4(pwm[4], servo_default_config_esc());
    // Servo sim_servo_5(pwm[5], servo_default_config_esc());
    // Servo sim_servo_6(pwm[6], servo_default_config_esc());
    // Servo sim_servo_7(pwm[7], servo_default_config_esc());

    // // Create MAV using simulated sensors
    // LEQuad::conf_t mav_config = LEQuad::default_config(MAVLINK_SYS_ID);

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
    // LEQuad mav = LEQuad( sim_imu,
    //                      sim.barometer(),
    //                      sim.gps(),
    //                      sim.sonar(),
    //                      board.uart0,                // mavlink serial
    //                      board.spektrum_satellite,
    //                      board.state_display_megafly_rev4_,
    //                      board.file_flash,
    //                      sim_battery,
    //                      sim_servo_0,
    //                      sim_servo_1,
    //                      sim_servo_2,
    //                      sim_servo_3 ,
    //                      sim_servo_4,
    //                      sim_servo_5,
    //                      sim_servo_6,
    //                      sim_servo_7 ,
    //                      file_log,
    //                      file_stat,
    //                      mav_config );

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
