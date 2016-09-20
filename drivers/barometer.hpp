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
 * \file barometer.hpp
 *
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 * \author Julien Lecoeur
 *
 * \brief Interface class for barometers
 *
 ******************************************************************************/


#ifndef BAROMETER_HPP_
#define BAROMETER_HPP_

#include <cstdint>

/**
 * \brief   Interface class for barometers
 */
class Barometer
{
public:
    Barometer();

    /**
     * \brief   Initialise the sensor
     *
     * \return  Success
     */
    virtual bool init(void) = 0;


    /**
     * \brief   Main update function
     * \detail  Reads new values from sensor
     *
     * \return  Success
     */
    virtual bool update(void) = 0;


    /**
    * \brief   Get the last update time in microseconds
    *
    * \return   Value
    */
    virtual uint64_t last_update_us(void) const = 0;


    /**
     * \brief   Return the pressure
     *
     * \return  Value
     */
    virtual float pressure(void)  const = 0;


    /**
     * \brief   Get the altitude in meters above sea level
     *
     * \detail  Global frame: (>0 means upward)
     *
     * \return  Value
     */
    virtual float altitude_gf(void) const = 0;


    /**
     * \brief   Get the vertical speed in meters/second
     *
     * \detail  NED frame: (>0 means downward)
     *
     * \return  Value
     */
    virtual float vertical_speed_lf(void) const = 0;


    /**
     * \brief   Get sensor temperature
     *
     * \return  Value
     */
    virtual float temperature(void) const = 0;


    /**
     * \brief   Gets the flag stating if the barometer has been calibrated
     *
     * \return  has_been_read_
     */
    bool has_been_calibrated() const;


    /**
     * \brief   Correct altitude offset using current altitude
     *
     * \param   current_altitude_gf     Current altitude in global frame
     */
    void calibrate_bias(float current_altitude_gf);


    /**
     * \brief   Compue altitude above sea level from pressure
     *
     * \param   pressure        Current atmospheric pressure
     * \param   altitude_bias   Altitude correction (optional)
     *
     * \return  Altitude (global frame)
     *
     */
    static float altitude_from_pressure(float pressure, float altitude_bias = 0);

protected:

    float altitude_bias_gf_;    ///< Offset of the barometer sensor for matching GPS altitude value
    bool has_been_calibrated_;  ///< Flag stating if the barometer has been calibrated
};

/**
 * \brief  Glue method for scheduler
 */
static inline bool task_barometer_update(Barometer* barometer)
{
    return barometer->update();
};

#endif /* BAROMETER_HPP_ */
