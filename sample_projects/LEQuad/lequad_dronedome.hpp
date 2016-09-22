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
 * \file lequad_dronedome.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class for indoor use
 *
 ******************************************************************************/


#ifndef LEQUAD_DRONEDOME_HPP_
#define LEQUAD_DRONEDOME_HPP_

#include "sample_projects/LEQuad/lequad.hpp"
#include "drivers/gps_mocap.hpp"
#include "sensing/ahrs_ekf_mocap.hpp"

/**
 * \brief Central data for indoor use
 */
class LEQuad_dronedome: public LEQuad
{
public:
    /**
     * \brief   Default configuration
     *
     * \param   sysid       System id (default value = 1)
     *
     * \return  Config structure
     */
    static inline LEQuad::conf_t default_config(uint8_t sysid = 1);

    /**
     * \brief   Constructor
     */
    LEQuad_dronedome( Imu& imu,
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
                      LEQuad::conf_t config = LEQuad_dronedome::default_config() ):
          LEQuad(imu, barometer, gps_mocap_, sonar, serial_mavlink, satellite, state_display, file_flash,
                     battery, servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7,
                     file1, file2, config),
          gps_mocap_(communication.handler()),
          ahrs_ekf_mocap_(communication.handler(), ahrs_ekf)
      {

      }

    /*
     * \brief   Initializes LEQuad
     * \details  Calls all init functions (init_*());
     *
     * \return  success
     */
    virtual bool init(void)
    {
        bool success = true;
        success &= LEQuad::init();
        success &= gps_mocap_.init();
        success &= ahrs_ekf_mocap_.init();
        return success;
    }

private:
    Gps_mocap gps_mocap_;
    Ahrs_ekf_mocap ahrs_ekf_mocap_;
};

LEQuad::conf_t LEQuad_dronedome::default_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf = LEQuad::default_config(sysid);

    //adapt gain for the drone dome
    for (int i = 0; i < 3; ++i)
    {
        conf.position_estimation_config.kp_pos_gps[i] = 100.0f;
        conf.position_estimation_config.kp_vel_gps[i] = 100.0f;
    }
    conf.position_estimation_config.kp_alt_baro = 0.0f;
    conf.position_estimation_config.kp_vel_baro = 0.0f;
    conf.position_estimation_config.kp_alt_sonar = 0.0f;
    conf.position_estimation_config.kp_vel_sonar = 0.0f;

    return conf;
}

#endif /* LEQUAD_DRONEDOME_HPP_ */
