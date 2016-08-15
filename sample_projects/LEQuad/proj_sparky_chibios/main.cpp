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

extern "C"
{
#include "hal.h"
}

#include "hal/chibios/i2c_chibios.hpp"

/* I2C interface #1 */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

/*
 * Application entry point.
 */
int main(void)
 {
    /*
    * System initializations.
    * - HAL initialization, this also initializes the configured device drivers
    *   and performs the board-specific initializations.
    */
    halInit();

    /*
    * Enabling interrupts, initialization done.
    */
    osalSysEnable();

    /*
    * Starts I2C
    */
    I2c_chibios::conf_t i2c_config =
    {
        .driver  = &I2CD1,
        .config  = i2cfg1,
        .timeout = 1000,
    };
    I2c_chibios i2c(i2c_config);
    i2c.init();

    /**
    * Prepares the barometer
    */
    static uint8_t rxbuf[6];
    static uint8_t txbuf[4];

    const uint8_t ms5611_addr       = 0x77;
    const uint8_t COMMAND_RESET     = 0x1E;  ///< Reset sensor
    const uint8_t COMMAND_GET_CALIBRATION          = 0xA2;  ///< Get 16bytes factory configuration from PROM

    txbuf[0] = 0xAF; /* register address */
    txbuf[1] = 0x1;
    // i2c.write(txbuf, 2, ms5611_addr);

    /*
    * Normal main() thread activity, in this demo it just performs
    * a shell respawn upon its termination.
    */
    while (true)
    {
        osalThreadSleepMilliseconds(500);
        palSetPad(GPIOB, GPIOB_PIN4);       /* Orange.  */
        osalThreadSleepMilliseconds(500);
        palClearPad(GPIOB, GPIOB_PIN4);     /* Orange.  */

        // Reset sensor
        txbuf[0] = COMMAND_RESET;
        i2c.write(txbuf, 1, ms5611_addr);

        // Let time to the sensor to initialize
        osalThreadSleepMilliseconds(20);

        // Read calibration data
        for (size_t i = 0; i < 6; i++)
        {
            txbuf[0] = COMMAND_GET_CALIBRATION + i * 2;
            i2c.transfer(txbuf, 1, rxbuf, 2, ms5611_addr);
        }
    }

     return 0;
 }
