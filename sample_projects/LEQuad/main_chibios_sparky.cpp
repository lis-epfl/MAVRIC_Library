/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "boards/sparky_chibi/sparky_chibi.hpp"
#include "sample_projects/LEQuad/lequad.hpp"

#include "hal/common/time_keeper.hpp"

#include "hal/dummy/serial_dummy.hpp"
#include "hal/dummy/i2c_dummy.hpp"
#include "hal/dummy/file_dummy.hpp"
#include "hal/dummy/adc_dummy.hpp"
#include "hal/dummy/pwm_dummy.hpp"

#include "simulation/dynamic_model_quad_diag.hpp"
#include "simulation/simulation.hpp"

#include "drivers/spektrum_satellite.hpp"

#include "drivers/lsm330dlc.hpp"
#include "drivers/hmc5883l.hpp"
#include "drivers/gps_ublox.hpp"
#include "drivers/sonar_i2cxl.hpp"


#define MAVLINK_SYS_ID 2

int main(void)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    Sparky_chibi::conf_t board_config = Sparky_chibi::default_config();
    Sparky_chibi board(board_config);

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
    Led_gpio            led_dummy(gpio_dummy);
    State_display_sparky_v2 state_display_dummy(led_dummy, led_dummy);

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
    Imu     sim_imu( sim.accelerometer(),
                     sim.gyroscope(),
                     sim.magnetometer() );


    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    // Create MAV using real sensors
    LEQuad::conf_t mav_config = LEQuad::default_config(MAVLINK_SYS_ID);
    LEQuad mav = LEQuad( sim_imu,
                         sim.barometer(),
                         sim.gps(),
                         sim.sonar(),
                         serial_dummy,                // mavlink serial
                         satellite_dummy,
                         state_display_dummy,
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
    // Main loop
    // -------------------------------------------------------------------------
    // mav.loop();








    I2c_chibios& i2c = board.i2c1_;

    Gpio_chibios gpio({GPIOB, GPIOB_PIN4});

    /**
    * Prepares the barometer
    */
    static uint8_t rxbuf[6];
    static uint8_t txbuf[4];

    const uint8_t ms5611_addr       = 0x77;
    const uint8_t COMMAND_RESET     = 0x1E;  ///< Reset sensor
    const uint8_t COMMAND_GET_CALIBRATION          = 0xA2;  ///< Get 16bytes factory configuration from PROM

    /*
    * Normal main() thread activity, in this demo it just performs
    * a shell respawn upon its termination.
    */
    while (true)
    {
        time_keeper_delay_ms(500);
        gpio.set_high();
        time_keeper_delay_ms(500);
        gpio.set_low();
        time_keeper_delay_ms(500);
        gpio.toggle();
        time_keeper_delay_ms(100);
        gpio.toggle();
        time_keeper_delay_ms(100);

        // Reset sensor
        txbuf[0] = COMMAND_RESET;
        i2c.write(txbuf, 1, ms5611_addr);

        // Let time to the sensor to initialize
        time_keeper_delay_ms(20);

        // Read calibration data
        for (size_t i = 0; i < 6; i++)
        {
            txbuf[0] = COMMAND_GET_CALIBRATION + i * 2;
            i2c.transfer(txbuf, 1, rxbuf, 2, ms5611_addr);
        }

        // scheduler.
        // scheduler.add_task(1000, (Scheduler_task::task_function_t)&time_keeper_init, (void*)&sim_imu);

    }

     return 0;
 }
