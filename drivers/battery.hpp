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
 * \file battery.hpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *
 * \brief Battery voltage monitor
 *
 ******************************************************************************/


#ifndef BATTERY_HPP_
#define BATTERY_HPP_


#include <cstdbool>
#include <cstdint>

#include "hal/common/adc.hpp"

/**
 * \brief Battery type
 */
typedef enum
{
    BATTERY_LIPO_1S,
    BATTERY_LIPO_2S,
    BATTERY_LIPO_3S,
    BATTERY_LIPO_4S,
    BATTERY_LIFE_1S,
    BATTERY_LIFE_2S,
    BATTERY_LIFE_3S,
    BATTERY_LIFE_4S,
} battery_type_t;


/**
 * \brief Configuration for Battery
 */
typedef struct
{
    battery_type_t  type;               ///< Battery type
    float           low_level_limit;    ///< Level from which the battery is considered low (in percents)
    float           lpf_gain;           ///< Low pass filter gain
} battery_conf_t;


/**
 * \brief Defautl configuration
 */
static inline battery_conf_t battery_default_config();


/**
 * \brief Battery voltage monitor
 */
class Battery
{
public:
    /**
     * \brief Constructor
     *
     * \param   adc     Reference to analog to digital converter
     * \param  config  Configuration
     */
    Battery(Adc& adc, battery_conf_t config = battery_default_config());


    /**
     * \brief   Updates the battery voltage level
     *
     * \param   battery   Pointer to the battery structure
     * \param   voltage   Update measured battery voltage
     *
     * \return Success
     */
    bool update(void);


    /**
     * \brief   Returns the level of the battery in volts
     *
     * \return  Voltage of the battery
     */
    const float& voltage(void) const;


    /**
     * \brief   Returns the level of the battery in percents (from 0 to 100)
     *
     * \return  Level of the battery
     */
    const float& level(void) const;


    /**
     * \brief   Indicates if the battery is low
     *
     * \return  True if low
     */
    bool is_low(void) const;


private:
    Adc&            adc_;               ///< Reference to analog to digital converter

    battery_conf_t  config_;            ///< Configuration

    float           voltage_;           ///< Current voltage of the battery in V
    float           level_;             ///< Current level of the battery in % (from 0 to 100)
    float           last_update_us_;    ///< Last update time in microseconds
    bool            is_low_;            ///< Indiciates if the battery level is low
};



static inline battery_conf_t battery_default_config()
{
    battery_conf_t conf = {};

    conf.type               = BATTERY_LIPO_3S;
    conf.low_level_limit    = 13.3f;
    conf.lpf_gain           = 0.5f;

    return conf;
}

#endif // BATTERY_HPP_
