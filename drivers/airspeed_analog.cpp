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
 * \file airspeed_analog.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Simon Pyroth
 *   
 * \brief This file is the driver for the DIYdrones airspeed sensor V20 
 * (old analog version)
 *
 ******************************************************************************/


#include "airspeed_analog.hpp"


extern "C"
{
#include "util/maths.h"
#include "hal/common/time_keeper.hpp"
}

#define AIRSPEED_SENSOR_OFFSET 2.5f         ///< Offset of the sensor [V]
#define AIRSPEED_SENSOR_SENSITIVITY 0.001f  ///< Sensitivity of the sensor [V/Pa]

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

float Airspeed_analog::get_raw_differential_pressure(void)
{
    /* Sensor: V = S*P + O
    V: Measured voltage [0;5 V]
    S: Sensitivity [0.001 V/Pa]
    P: Differential pressure [Pa]
    O: Offset [2.5 V]
    
    ==> P = (V - O)/S
    */
    
    voltage = adc_.voltage();
    float P = (voltage - AIRSPEED_SENSOR_OFFSET)/AIRSPEED_SENSOR_SENSITIVITY;
    
    return  P;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Airspeed_analog::Airspeed_analog(Adc& adc, airspeed_analog_conf_t config):
    adc_(adc),
    config_(config)
{
    
    // Default values to start the calibration correctly !
    voltage = 0.0f;
    differential_pressure = 0.0f;
    raw_airspeed = 0.0f;
    scaled_airspeed = 0.0f;
    airspeed = 0.0f;
    last_airspeed = 0.0f;
    calibrating = false;
    
    // Start calibration
    start_calibration();
}

void Airspeed_analog::start_calibration(void)
{
    // Trigger the calibration
    calibrating = true;
}

void Airspeed_analog::stop_calibration(void)
{
    // Stop calibration process
    calibrating = false;
}


bool Airspeed_analog::update(void)
{
    // Get differential pressure
    float raw_differential_pressure = get_raw_differential_pressure();

    // Wait before ADC reads correct values for pressure offset (compute it directly on value coming out of the sensor)
    if (calibrating && time_keeper_get_s() >= 3)
    {
        // Mean value of the raw airspeed --> low-pass filter !
        config_.pressure_offset = config_.calibration_gain * config_.pressure_offset + (1 - config_.calibration_gain) * raw_differential_pressure;
    }

    // Correct the raw pressure
    differential_pressure = raw_differential_pressure - config_.pressure_offset;
    
    // First airspeed estimation, avoiding negative pressure
    // TODO: Plug the tube in the correct way and use max(pres, 0) instead of abs to have better results !
    raw_airspeed = maths_fast_sqrt( maths_f_abs(differential_pressure) * config_.conversion_factor );
    
    // Correct it using fitted relation: Airspeed_measured = gain * Airspeed_true + offset
    scaled_airspeed = (raw_airspeed - config_.correction_offset)/config_.correction_gain;
    
    // Filter the airspeed
    if (time_keeper_get_s() >= 3)
    {
        last_airspeed = airspeed;
        airspeed = config_.filter_gain * airspeed + (1 - config_.filter_gain) * scaled_airspeed;
    }

    // TODO: Convert to true airspeed (EAS --> TAS) so that convertion_factor is no more dependent on altitude !
    
    return true;
}


const float& Airspeed_analog::get_airspeed() const
{
    return airspeed;
}

const float& Airspeed_analog::get_last_airspeed() const
{
    return last_airspeed;
}
