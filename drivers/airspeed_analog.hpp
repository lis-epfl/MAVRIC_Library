/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file airspedd_analog.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Simon Pyroth
 *   
 * \brief This file is the driver for the DIYdrones airspeed sensor V20 
 * (old analog version)
 *
 ******************************************************************************/


#ifndef AIRSPEED_ANALOG_H_
#define AIRSPEED_ANALOG_H_

#include <stdint.h>
#include "hal/common/adc.hpp"



/**
 * \brief  Configuration for the airspeed sensor
 */
typedef struct 
{
    float pressure_offset;          ///< Default airspeed offset
    float calibration_gain;         ///< Gain used for the calibration of the offset (low-pass)
    float conversion_factor;        ///< Factor used for conversion between differential pressure P and square speed v^2. Should be around 2/rho_air. This parameter is the one to tune the sensor !
    float correction_gain;          ///< Gain obtained by the fitted relation, if needed (Airspeed_measured = gain * Airspeed_true + offset)
    float correction_offset;        ///< Offset obtained by the fitted relation, if needed
    float filter_gain;              ///< Gain for the low-pass filter
} airspeed_analog_conf_t;

static inline airspeed_analog_conf_t airspeed_analog_default_config();

class Airspeed_analog
{
public:
    /**
     * \brief Constructor
     *
     * \param  adc     Reference to analog to digital converter
     * \param  config  Configuration
     */
    Airspeed_analog(Adc& adc, airspeed_analog_conf_t config = airspeed_analog_default_config());


    /**
     * \brief Initialize the airspeed sensor
     *
     * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
     * \param analog_monitor pointer to the structure of analog monitor module
     * \param analog_channel set which channel of the ADC is map to the airspeed sensor
     *
    */
    bool init(const airspeed_analog_conf_t* config);

    /**
     * \brief Calibrates the airspeed sensor offset at 0 speed. It will continue calibrating until it is asked to stop.
     * 
     * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
     *
    */
    void start_calibration(void);

    /**
     * \brief Stop the calibration procedure and keep the last offset.
     * 
     * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
     *
    */
    void stop_calibration(void);

    /**
     * \brief Updates the values in the airspeed structure
     *
     * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
     *
     * \return  The result of the task execution
    */
    bool update(void);

    /**
     * \brief Gets the current estimated airspeed
     *
     * \return  The current estimated airspeed
    */
    const float& get_airspeed() const;

private:
    /**
     * \brief Returns the raw differential pressure measured by the airspeed sensor in Pascal
     *
     * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
     *
     * \return the raw differential pressure measured by the sensor (in Pa)
    */
    float get_raw_differential_pressure(void);

    Adc& adc_;                              ///< Reference to analog to digital converter
    
    airspeed_analog_conf_t config_;         ///< Configuration

    float voltage;                          ///< Voltage read by the ADC
    
    float differential_pressure;            ///< True differential pressure in Pa (raw sensor compensated with offset)
    
    float raw_airspeed;                     ///< Unfiltered and uncorrected airspeed
    float scaled_airspeed;                  ///< Corrected airspeed, using fitted relation
    float airspeed;                         ///< Filtered corrected airspeed
    float last_airspeed;                    ///< Airspeed from previous loop
    
    bool calibrating;                       ///< True if the sensor is currently in calibration, false otherwise

};



static inline airspeed_analog_conf_t airspeed_analog_default_config()
{
    airspeed_analog_conf_t conf = {};

    conf.pressure_offset = 0.0f;
    conf.calibration_gain = 0.9f;
    conf.conversion_factor = 0.3945f;
    conf.correction_gain = 1.0f;
    conf.correction_offset = 0.0f;
    conf.filter_gain = 0.7f;

    return conf;
}

#endif /* AIRSPEED_ANALOG_H_ */