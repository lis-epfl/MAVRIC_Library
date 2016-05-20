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
 * \file lequad_merk.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#ifndef LEQUAD_MERK_HPP_
#define LEQUAD_MERK_HPP_

#include "sample_projects/LEQuad/lequad.hpp"
#include "sample_projects/LEQuad/saccade_controller.hpp"
#include "drivers/gps_mocap.hpp"
#include "drivers/flow.hpp"
/**
 * \brief MAV class
 */
class LEQuad_merk: public LEQuad
{
public:
    /**
     * \brief   Configuration structure
     */
     struct conf_t
    {
        LEQuad::conf_t lequad_config;
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
    LEQuad_merk( Imu& imu,
            Barometer& barometer,
            Gps& gps,
            Sonar& sonar,
            Serial& serial_mavlink,
            Satellite& satellite,
            Led& led,
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
            Flow& flow1,
            Flow& flow2,
            const conf_t& config = default_config());

protected:
    virtual bool main_task(void);
    virtual bool init_saccade(void);

    Gps_mocap gps_mocap_;
    Saccade_controller saccade_controller_;
};


LEQuad_merk::conf_t LEQuad_merk::default_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf.lequad_config = LEQuad::default_config();

    return conf;
};

#endif /* LEQUAD_MERK_HPP_ */
