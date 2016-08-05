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
    enum state_t
    {
        IDLE,                   ///< Is doing nothing
        SAMPLING_TEMPERATURE,   ///< Is sampling temperature
        SAMPLING_PRESSURE,      ///< Is sampling pressure
    };


    /**
     * \brief   Constructor
     *
     * \param   i2c     Reference to I2C device
     */
    Barometer_MS5611(I2c& i2c);


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
    state_t     state_;                 ///< Sensor state
    float       last_state_update_us_;  ///< Time of the last state update
    float       last_update_us_;        ///< Time of the last update
    float       dt_s_;                  ///< Time step for the derivative

    float   pressure_;              ///< Measured pressure
    float   temperature_;           ///< Measured temperature
    float   altitude_gf_;           ///< Measured altitude (global frame)
    float   speed_lf_;              ///< Vario altitude speed (ned frame)

    float       last_altitudes_[3];     ///< Array to store previous value of the altitude for low pass filtering the output
};

#endif /* BMP085_HPP_ */
