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
 * \file lequad.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#ifndef LEQUAD_HPP_
#define LEQUAD_HPP_

#include "drones/mav.hpp"
#include "flight_controller/flight_controller_quadcopter_diag.hpp"

/**
 * \brief MAV class
 */
class LEQuad: public MAV
{
public:

    /**
     * \brief   Configuration structure
     */
    struct conf_t
    {
        MAV::conf_t mav_config;
        Flight_controller_quadcopter_diag::conf_t flight_controller_config;
    };

    /**
     * \brief   Default configuration
     *
     * \param   sysid       System id (default value = 1)
     *
     * \return  Config structure
     */
    static inline conf_t default_config(uint8_t sysid = 1);


    /**
     * \brief   Configuration for use in drone dome
     *
     * \param   sysid       System id (default value = 1)
     *
     * \return  Config structure
     */
    static inline conf_t dronedome_config(uint8_t sysid = 1);


    /**
     * \brief   Constructor
     */
    LEQuad( Imu& imu,
            Barometer& barometer,
            Gps& gps,
            Sonar& sonar,
            Px4flow_i2c& flow,
            Serial& serial_mavlink,
            Satellite& satellite,
            State_display& state_display,
            File& file_flash,
            Battery& battery,
            File& file1,
            File& file2,
            Servo& servo_0,
            Servo& servo_1,
            Servo& servo_2,
            Servo& servo_3,
            const conf_t& config = default_config());

    bool init_controller(void);

protected:
    Flight_controller_quadcopter_diag    flight_controller_quadcopter_diag_;
};


LEQuad::conf_t LEQuad::default_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf.mav_config = MAV::default_config();
    conf.flight_controller_config = Flight_controller_quadcopter_diag::default_config();

    return conf;
};


LEQuad::conf_t LEQuad::dronedome_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf.mav_config = MAV::dronedome_config(sysid);
    conf.flight_controller_config = Flight_controller_quadcopter_diag::default_config();

    return conf;
}

#endif /* LEQUAD_HPP_ */
