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

#include "hal/stm32/spi_stm32.hpp"

#include "hal/dummy/serial_dummy.hpp"
#include "hal/dummy/i2c_dummy.hpp"
#include "hal/dummy/file_dummy.hpp"
#include "hal/dummy/adc_dummy.hpp"
#include "hal/dummy/pwm_dummy.hpp"

#include "simulation/dynamic_model_quad_diag.hpp"
#include "simulation/simulation.hpp"

#include "drivers/spektrum_satellite.hpp"
#include "util/string_util.hpp"

 #include "drivers/mpu_9250.hpp"

extern "C"
{
#include "util/print_util.h"
}

#define MAVLINK_SYS_ID 2

int main(int argc, char** argv)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    sparky_v2_conf_t board_config = sparky_v2_default_config();
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

    // -------------------------------------------------------------------------
    // Create simulation
    // -------------------------------------------------------------------------
    // Simulated servos
    Pwm_dummy pwm[8];
    Servo sim_servo_0(pwm[0], servo_default_config_esc());
    Servo sim_servo_1(pwm[1], servo_default_config_esc());
    Servo sim_servo_2(pwm[2], servo_default_config_esc());
    Servo sim_servo_3(pwm[3], servo_default_config_esc());
    Servo sim_servo_4(pwm[4], servo_default_config_esc());
    Servo sim_servo_5(pwm[5], servo_default_config_esc());
    Servo sim_servo_6(pwm[6], servo_default_config_esc());
    Servo sim_servo_7(pwm[7], servo_default_config_esc());

    // Simulated dynamic model
    Dynamic_model_quad_diag     sim_model(sim_servo_0, sim_servo_1, sim_servo_2, sim_servo_3);
    Simulation                  sim(sim_model);

    // Simulated battery
    Adc_dummy   sim_adc_battery(11.1f);
    Battery     sim_battery(sim_adc_battery);

    // Simulated IMU
    // Imu     sim_imu( sim.accelerometer(),
    //                  sim.gyroscope(),
    //                  sim.magnetometer() );

    // set the flag to simulation
    LEQuad::conf_t mav_config = LEQuad::default_config(MAVLINK_SYS_ID);
    LEQuad mav = LEQuad( board.imu_,
                         //sim_imu,
                         sim.barometer(),
                         sim.gps(),
                         sim.sonar(),
                         board.serial_,                // mavlink serial
                         satellite_dummy,
                         board.state_display_sparky_v2_,
                        file_dummy,
                         sim_battery,
                         sim_servo_0,
                         sim_servo_1,
                         sim_servo_2,
                         sim_servo_3 ,
                         sim_servo_4,
                         sim_servo_5,
                         sim_servo_6,
                         sim_servo_7 ,
                         file_dummy,
                         file_dummy,
                         mav_config );

    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    // Create MAV using real sensors
    // LEQuad::conf_t mav_config = LEQuad::default_config(MAVLINK_SYS_ID);
    // LEQuad mav = LEQuad(board.imu,
    //                     board.bmp085,
    //                     board.gps_ublox,
    //                     board.sonar_i2cxl,
    //                     board.uart0,
    //                     board.spektrum_satellite,
    //                     board.state_display_megafly_rev4_,
    //                     board.file_flash,
    //                     board.battery,
    //                     board.servo_0,
    //                     board.servo_1,
    //                     board.servo_2,
    //                     board.servo_3,
    //                     board.servo_4,
    //                     board.servo_5,
    //                     board.servo_6,
    //                     board.servo_7,
    //                     file_dummy,
    //                     file_dummy,
    //                     mav_config );



    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    mav.loop();

    // board.led_err_.off();
    // board.led_stat_.off();
    // board.led_rf_.off();

    // Console<Serial> console(board.serial_);

    // //Mpu_9250 mpu(board.spi_1_);
    // //bool bo = mpu.init();
    // bool bo = true;


    // // Mpu_9250& mpu = board.mpu_9250_;
    // Mpu_9250& mpu = board.mpu_9250_;
    // bo &= mpu.init();

    // uint8_t mag_com_en[] = {mpu.MPU9250_USER_CTRL_REG, (uint8_t)(mpu.MPU9250_USERCTL_I2C_MST_EN |
    //                                                          mpu.MPU9250_USERCTL_DIS_I2C)};

    // bo &= mpu.write_reg(mag_com_en);
    // time_keeper_delay_ms(100);

    // // mag_com_en[1] = mpu.MPU9250_USERCTL_I2C_MST_EN;
    // // bo &= mpu.write_reg(mag_com_en);

    // uint8_t whoami_id[2]      = {0};
    // // Read WhoAmI for Mag
    // bo &= mpu.mag_read_reg(mpu.AK8963_WHOAMI_REG, whoami_id);
    // bo &= whoami_id[1] == mpu.AK8963_WHOAMI_ID ? true : false;
    // bo &= mpu.mag_reset();

    // uint8_t ctrl_cmd[] = {mpu.AK8963_CNTL1_REG, 0x16};
    // bo &= mpu.mag_write_reg(ctrl_cmd);
    // uint8_t ctrl_cmd_val[] = {0};
    // bo &= mpu.mag_read_reg(mpu.AK8963_CNTL1_REG, ctrl_cmd_val);
    // bo &= ctrl_cmd_val[1] == 0x16 ? true : false;

    // uint8_t slv0_reg_reg[]  = {mpu.MPU9250_SLV0_REG_REG, mpu.AK8963_ST1_REG};
    // uint8_t slv0_addr_reg[] = {mpu.MPU9250_SLV0_ADDR_REG, (uint8_t)(mpu.MPU9250_AK8963_ADDR | mpu.MPU9250_READ_FLAG)};
    // uint8_t slv0_ctrl_reg[] = {mpu.MPU9250_SLV0_CTRL_REG, (uint8_t)(mpu.MPU9250_I2CSLV_EN | 8)}; // 8 bytes ??

    // bo &= mpu.write_reg(slv0_reg_reg);
    // bo &= mpu.write_reg(slv0_addr_reg);
    // bo &= mpu.write_reg(slv0_ctrl_reg);

    // Imu imu(mpu, mpu, mpu);

    // while (1)
    // {
    //     //board.state_display_sparky_v2_.update();
    //     // Warning: if too short serial does not work
    //     //time_keeper_delay_ms(1000);


    //     // Write mavlink message
    //     // mavlink_msg_heartbeat_pack( 11,     // uint8_t system_id,
    //     //                             50,     // uint8_t component_id,
    //     //                             &msg,   // mavlink_message_t* msg,
    //     //                             0,      // uint8_t type,
    //     //                             0,      // uint8_t autopilot,
    //     //                             0,      // uint8_t base_mode,
    //     //                             0,      // uint32_t custom_mode,
    //     //                             0);     //uint8_t system_status)
    //     // mavlink_stream.send(&msg);

    //     // time_keeper_delay_ms(25);
    //     time_keeper_delay_ms(500);


    //     // mpu.update_acc();
    //     // mpu.update_gyr();
    //     // mpu.update_mag();
    //     // bo = imu.update();
    //     imu.update();

    //     uint8_t out_test[] = {mpu.MPU9250_USER_CTRL_REG, 0};
    //     uint8_t in_test[] = {0,0};
    //     bo &= mpu.read_reg(out_test, in_test);
    //     bo &= in_test[1] == (uint8_t)(mpu.MPU9250_USERCTL_I2C_MST_EN | mpu.MPU9250_USERCTL_DIS_I2C) ? true : false; // DIS is not reset

    //     bo &= mpu.mag_read_reg(mpu.AK8963_WHOAMI_REG, whoami_id);
    //     bo &= whoami_id[1] == mpu.AK8963_WHOAMI_ID ? true : false;

    //     uint8_t in_buffer[2] = {0};
    //     // bo &= mpu.mag_read_reg(0x02, in_buffer);
    //     // bo &= in_buffer[1] == 0x00 ? true : false;

    //     // bo &= mpu.mag_read_reg(0x03, in_buffer);
    //     // uint8_t xl = in_buffer[1];
    //     // bo &= mpu.mag_read_reg(0x04, in_buffer);

    //     // bo &= mpu.mag_read_reg(0x02, in_buffer);
    //     // bo &= in_buffer[1] == 0x00 ? true : false;

    //     // bo &= mpu.mag_read_reg(0x09, in_buffer);
    //     // bo &= in_buffer[1] == 0x00 ? true : false;

    //     // ctrl_cmd[1] = 0x16;
    //     // bo &= mpu.mag_write_reg(ctrl_cmd);
    //     // bo &= mpu.mag_read_reg(mpu.AK8963_CNTL1_REG, ctrl_cmd_val);
    //     // bo &= ctrl_cmd_val[1] == 0x16 ? true : false;

    //     // bo &= mpu.mag_read_reg(0x05, in_buffer);
    //     // uint8_t yl = in_buffer[1];
    //     // bo &= mpu.mag_read_reg(0x06, in_buffer);

    //     // bo &= mpu.mag_read_reg(0x09, in_buffer);
    //     // bo &= in_buffer[1] == 0x00 ? true : false;

    //     // ctrl_cmd[1] = 0x16;
    //     // bo &= mpu.mag_write_reg(ctrl_cmd);
    //     // bo &= mpu.mag_read_reg(mpu.AK8963_CNTL1_REG, ctrl_cmd_val);
    //     // bo &= ctrl_cmd_val[1] == 0x16 ? true : false;

    //     // bo &= mpu.mag_read_reg(0x07, in_buffer);
    //     // uint8_t zl = in_buffer[1];
    //     // bo &= mpu.mag_read_reg(0x08, in_buffer);

    //     // bo &= mpu.mag_read_reg(0x09, in_buffer);
    //     // bo &= in_buffer[1] == 0x00 ? true : false;

    //     // ctrl_cmd[1] = 0x16;
    //     // bo &= mpu.mag_write_reg(ctrl_cmd);
    //     // bo &= mpu.mag_read_reg(mpu.AK8963_CNTL1_REG, ctrl_cmd_val);
    //     // bo &= ctrl_cmd_val[1] == 0x16 ? true : false;

    //     // int16_t accx = (int16_t)mpu.acc_X();
    //     // int16_t accy = (int16_t)mpu.acc_Y();
    //     // int16_t accz = (int16_t)mpu.acc_Z();

    //     // int16_t gyrx = (int16_t)mpu.gyro_X();
    //     // int16_t gyry = (int16_t)mpu.gyro_Y();
    //     // int16_t gyrz = (int16_t)mpu.gyro_Z();

    //     // int16_t magx = (int16_t)mpu.mag_X();
    //     // int16_t magy = (int16_t)mpu.mag_Y();
    //     // int16_t magz = (int16_t)mpu.mag_Z();

    //     // uint8_t in_buffer[2] = {0};
    //     // bo &= mpu.mag_read_reg(0x03, in_buffer);
    //     // uint8_t xl = in_buffer[1];
    //     // bo &= mpu.mag_read_reg(0x04, in_buffer);
    //     // int16_t valx = (int16_t)(in_buffer[1] << 8 | xl);//accx;
    //     // int16_t valy = (int16_t)(in_buffer[1] << 8 | yl);//accy;
    //     // int16_t valz = (int16_t)(in_buffer[1] << 8 | zl);//accz;

    //     // int16_t valx = magx;
    //     // int16_t valy = magy;
    //     // int16_t valz = magz;

    //     const char* sep = "\t";
    //     uint64_t delay = 25;//1; //ms

    //     uint8_t ser_buf[str::MAX_DIGITS10_LONG];
    //     uint8_t ser_len = 0;
    //     // str::format_integer(valx, ser_buf, &ser_len);

    //     // board.serial_.write(ser_buf, ser_len);
    //     //print_util_dbg_log_value("\r\n wai", val, 16);
    //     console.write(valx);
    //     time_keeper_delay_ms(delay);
    //     board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     time_keeper_delay_ms(delay);
    //     console.write(valy);
    //     time_keeper_delay_ms(delay);
    //     board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     time_keeper_delay_ms(delay);
    //     console.write(valz);
    //     time_keeper_delay_ms(delay);

    //     // board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     // time_keeper_delay_ms(delay);
    //     // board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     // time_keeper_delay_ms(delay);


    //     // console.write(gyrx);
    //     // time_keeper_delay_ms(delay);
    //     // board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     // time_keeper_delay_ms(delay);
    //     // console.write(gyry);
    //     // time_keeper_delay_ms(delay);
    //     // board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     // time_keeper_delay_ms(delay);
    //     // console.write(gyrz);
    //     // time_keeper_delay_ms(delay);

    //     // board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     // time_keeper_delay_ms(delay);
    //     // board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     // time_keeper_delay_ms(delay);
        
    //     // console.write(magx);
    //     // time_keeper_delay_ms(delay);
    //     // board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     // time_keeper_delay_ms(delay);
    //     // console.write(magy);
    //     // time_keeper_delay_ms(delay);
    //     // board.serial_.write((const uint8_t*)sep, sizeof(sep));
    //     // time_keeper_delay_ms(delay);
    //     // console.write(magz);
    //     // time_keeper_delay_ms(delay);

    //     const char* newline = "\r\n";
    //     board.serial_.write((const uint8_t*)newline, sizeof(newline));

    //     if (bo)
    //     {
    //         board.led_stat_.toggle();
    //     }
    //     else
    //     {
    //         board.led_err_.toggle();
    //     }

    // }

    return 0;
}
