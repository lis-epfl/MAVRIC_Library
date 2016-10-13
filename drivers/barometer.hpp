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
#include <cfloat>

/**
 * \brief   Interface class for barometers
 */
class Barometer
{
public:
    Barometer(float pressure_at_sea_level = 101325.0f);

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
     * \brief   Return the pressure (in Pa)
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
     * \brief   Get the pressure at sea level
     * \detail  The pressure at sea level is used to compute altitude from pressure
     *
     * \param   pressure_at_sea_level (in Pa)
     */
    static float pressure_at_sea_level(void);

    /**
     * \brief   Compute altitude above sea level from pressure
     *
     * \param   pressure                Current atmospheric pressure (in Pa)
     * \param   pressure_at_sea_level   Pressure at sea level (in Pa) (optional)
     *
     * \return  Altitude (global frame)
     */
    static float compute_altitude_from_pressure(float pressure, float pressure_at_sea_level = pressure_at_sea_level());


    /**
     * \brief   Compute pressure at sea level from pressure and altitude
     *
     * \param   pressure        Current atmospheric pressure (in Pa)
     * \param   altitude_       Altitude
     *
     * \return  Pressure at sea level (in Pa)
     */
    static float compute_pressure_at_sea_level(float pressure, float altitude);


protected:
    /**
     * \brief   Update the pressure at sea level
     * \detail  The pressure at sea level is used to compute altitude from pressure
     *
     * \param   pressure_at_sea_level
     */
    static void set_pressure_at_sea_level(float pressure_at_sea_level);


private:
    static float pressure_at_sea_level_;    ///< Pressure at sea level (is Pa). This is a static variable shared by all barometers
};

/**
 * \brief  Glue method for scheduler
 */
static inline bool task_barometer_update(Barometer* barometer)
{
    return barometer->update();
};

#endif /* BAROMETER_HPP_ */
