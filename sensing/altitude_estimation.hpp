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
 * \file altitude_estimation.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Altitude estimation
 *
 ******************************************************************************/


#ifndef ALTITUDE_ESTIMATION_HPP_
#define ALTITUDE_ESTIMATION_HPP_


#include "drivers/sonar.hpp"
#include "drivers/barometer.hpp"
#include "sensing/ahrs.hpp"
#include "util/kalman.hpp"

extern "C"
{
#include "sensing/altitude.h"
}


/**
 * \brief   Configuration structure
 */
typedef struct
{
} altitude_estimation_conf_t;


/**
 * \brief   Default configuration
 *
 * \return  Configuration structure
 */
static inline altitude_estimation_conf_t altitude_estimation_default_config(void);


/**
 * \brief   Altitude estimator
 */
class Altitude_estimation: public Kalman<3,1,1>
{
public:
    Altitude_estimation(Sonar& sonar,
                        Barometer& barometer,
                        ahrs_t& ahrs,
                        altitude_t& altitude,
                        altitude_estimation_conf_t config = altitude_estimation_default_config() );

    /**
     * \brief   Initialization
     *
     * \return  Success
     */
    bool init(void);


    /**
     * \brief   Main update function
     *
     * \return  Success
     */
    bool update(void);


private:
    const Sonar&        sonar_;           ///< Sonar, must be downward facing (input)
    const Barometer&    barometer_;       ///< Barometer (input)
    const ahrs_t&       ahrs_;            ///< Attitude and acceleration (input)
    altitude_t&         altitude_;        ///< Estimated altitude (output)

    altitude_estimation_conf_t config_;   ///< Configuration

    float last_sonar_update_us_;          ///< Last time we updated the estimate using sonar
};


static inline altitude_estimation_conf_t altitude_estimation_default_config(void)
{
    altitude_estimation_conf_t conf = {};

    return conf;
}

#endif /* ALTITUDE_ESTIMATION_HPP_ */
