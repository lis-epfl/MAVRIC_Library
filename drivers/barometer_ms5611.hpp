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
 * \file bmp085.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Driver for the MS5611 barometer
 *
 ******************************************************************************/


#ifndef BMP085_HPP_
#define BMP085_HPP_

#include <cstdint>
#include <cstdbool>

#include "drivers/barometer.hpp"
#include "hal/common/i2c.hpp"

/**
 * \brief   Driver for the BMP085 barometer
 */
class Barometer_MS5611: public Barometer
{
public:
    /**
     * \brief Sensor state
     */
    enum class state_t
    {
        INIT,                  ///< Sensor is not responding, will retry
        IDLE,                   ///< First run, start temperature sampling
        GET_TEMPERATURE,        ///< Read temperature from sensor, start pressure sampling
        GET_PRESSURE,           ///< Read pressure from sensor, start temperature sampling
    };

    /**
     * \bief I2C address of the sensor
     */
    enum class address_t
    {
        ADDR_CSBLOW  = 0x77,     ///< I2C address when CSB pin is connected to GND
        ADDR_CSBHIGH = 0x76,     ///< I2C address when CSB pin is connected to VCC
    };

    /**
     * \brief Oversampling ratio
     */
    enum class oversampling_ratio_t
    {
        OSR_256  = 0x00,            ///<  256 : resolution 0.065mbar, sampling time 0.54ms
        OSR_512  = 0x01,            ///<  512 : resolution 0.042mbar, sampling time 1.06ms
        OSR_1024 = 0x02,            ///< 1024 : resolution 0.027mbar, sampling time 2.08ms
        OSR_2048 = 0x03,            ///< 2048 : resolution 0.018mbar, sampling time 4.13ms
        OSR_4096 = 0x04,            ///< 4096 : resolution 0.012mbar, sampling time 8.22ms
    };

    struct calibration_data_t
    {
        uint16_t SENS_T1;
        uint16_t OFF_T1;
        uint16_t TCS;
        uint16_t TCO;
        uint16_t T_REF;
        uint16_t TEMPSENS;
    };

    static const uint8_t COMMAND_RESET                    = 0x1E;  ///< Reset sensor
    static const uint8_t COMMAND_GET_CALIBRATION          = 0xA2;  ///< Get 16bytes factory configuration from PROM
    static const uint8_t COMMAND_START_PRESSURE_CONV      = 0x40;  ///< Start pressure conversion (must add oversampling ratio to this command)
    static const uint8_t COMMAND_START_TEMPERATURE_CONV   = 0x50;  ///< Start temperature conversion (must add oversampling ratio to this command)
    static const uint8_t COMMAND_GET_DATA                 = 0x00;  ///< Get 3 bytes data (can be pressure or temperature depending of which sampling was started)

    /**
     * \brief Configuration structure
     */
    struct conf_t
    {
        address_t               address;
        oversampling_ratio_t    oversampling_ratio_pressure;
        oversampling_ratio_t    oversampling_ratio_temperature;
    };

    /**
     * \brief   Default Configuration
     *
     * \return  config
     */
    static inline conf_t default_config(void);

    /**
     * \brief   Constructor
     *
     * \param   i2c     Reference to I2C device
     */
    Barometer_MS5611(I2c& i2c, conf_t config = default_config());


    /**
     * \brief   Initialise the sensor
     *
     * \return  Success
     */
    bool init(void);


    /**
     * \brief   Main update function
     * \detail  Reads new values from sensor
     *
     * \return  Success
     */
    bool update(void);


    /**
    * \brief   Get the last update time in microseconds
    *
    * \return   Value
    */
    uint64_t last_update_us(void) const;


    /**
     * \brief   Return the pressure
     *
     * \return  Value
     */
    float pressure(void)  const;


    /**
     * \brief   Get the altitude in meters above sea level
     *
     * \detail  Global frame: (>0 means upward)
     *
     * \return  Value
     */
    float altitude_gf(void) const;


    /**
     * \brief   Get the vertical speed in meters/second
     *
     * \detail  NED frame: (>0 means downward)
     *
     * \return  Value
     */
    float vertical_speed_lf(void) const;


    /**
     * \brief   Get sensor temperature
     *
     * \return  Value
     */
    float temperature(void) const;


private:
    I2c&        i2c_;                   ///< Reference to I2C peripheral
    conf_t      config_;                ///< Configuration
    state_t     state_;                 ///< Sensor state

    calibration_data_t  calib_;          ///< Calibration data read from sensors PROM
    uint8_t             i2c_address;

    uint32_t time_sampling_start_ms_;

    float       last_state_update_us_;  ///< Time of the last state update
    float       last_update_us_;        ///< Time of the last update
    float       dt_s_;                  ///< Time step for the derivative
    float   pressure_;              ///< Measured pressure
    float   temperature_;           ///< Measured temperature
    float   altitude_gf_;           ///< Measured altitude (global frame)
    float   speed_lf_;              ///< Vario altitude speed (ned frame)

    float       last_altitudes_[3];     ///< Array to store previous value of the altitude for low pass filtering the output

    /**
     * \brief   Start temperature sampling
     *
     * \return  success
     */
    bool start_temperature_sampling(void);

    /**
     * \brief   Read pressure sampling
     *
     * \return  success
     */
    bool read_temperature(void);

    /**
     * \brief   Start pressure sampling
     *
     * \return  success
     */
    bool start_pressure_sampling(void);

    /**
     * \brief   Read pressure sampling
     *
     * \return  success
     */
    bool read_pressure(void);
};


/**
 * \brief   Default Configuration
 *
 * \return  config
 */
Barometer_MS5611::conf_t Barometer_MS5611::default_config(void)
{
    Barometer_MS5611::conf_t config = {};

    config.address                         = address_t::ADDR_CSBLOW;
    // config.oversampling_ratio_pressure     = oversampling_ratio_t::OSR_4096;
    // config.oversampling_ratio_temperature  = oversampling_ratio_t::OSR_4096;

    config.oversampling_ratio_pressure     = oversampling_ratio_t::OSR_256;
    config.oversampling_ratio_temperature  = oversampling_ratio_t::OSR_256;

    return config;
}

#endif /* BMP085_HPP_ */
