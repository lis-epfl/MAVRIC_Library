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
 * \file lequad_symbiotic.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#ifndef LEQUAD_SYMBIOTIC_HPP_
#define LEQUAD_SYMBIOTIC_HPP_

#include "sample_projects/LEQuad/lequad_dronedome.hpp"
#include "sample_projects/LEQuad/lequad.hpp"

#include "control/gimbal_controller.hpp"
#include "control/gimbal_controller_telemetry.hpp"

/**
 * \brief MAV class
 */


typedef LEQuad Base_class;


class LEQuad_symbiotic: public Base_class
{
public:
    /**
     * \brief   Configuration structure
     */
     struct conf_t
    {
        LEQuad::conf_t lequad_config;
        Gimbal_controller::conf_t gimbal_controller_config;
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
     * \brief   Constructor
     */
    LEQuad_symbiotic(
    		Imu& imu,
            Barometer& barometer,
            Gps& gps,
            Sonar& sonar,
            Serial& serial_mavlink,
            Satellite& satellite,
			State_display& state_display,
            File& file_flash,
            Battery& battery,
            Servo& servo_0,
            Servo& servo_1,
            Servo& servo_2,
            Servo& servo_3,
            Servo& servo_4,
            Servo& servo_5,
            Servo& servo_6,
            Servo& servo_7,
            File& file1,
            File& file2,
            const conf_t& config = default_config());

    //Fence configuration
	float z_min;//[m]
	float z_max;//[m]
	float xy_max; //radius [m]
	float dist_to_limit_lat; //[m]
	float dist_to_limit_ver; //[m]
	float _f_outside; //if 1, outside fence enabled, if 0, disabled

protected:

    Gimbal_controller				gimbal_controller;			///< The class gimbal control

    virtual bool main_task(void);
    virtual bool init_gimbal(void);
};


LEQuad_symbiotic::conf_t LEQuad_symbiotic::default_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf.lequad_config = LEQuad::default_config(sysid);
    conf.gimbal_controller_config = Gimbal_controller::default_config();

    //conf.lequad_config.navigation_config.takeoff_altitude = -15.0f;

    return conf;
};

#endif /* LEQUAD_SYMBIOTIC_HPP_ */
