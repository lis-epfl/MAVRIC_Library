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
 * \file saccade_controller.hpp
 *
 * \author Darius Merk
 *
 * \brief   Saccade controller for indoors collision free navigation based on optic flow
 *
 ******************************************************************************/


#ifndef SACCADE_CONTROLLER_HPP_
#define SACCADE_CONTROLLER_HPP_

extern "C"
{
#include "control/control_command.h"
#include "util/quaternions.h"
#include "util/coord_conventions.h"
}

#include "drivers/flow.hpp"

/**
 * \brief Configuration structure
 * \param   pitch               Pitch command for forward motion
 * \param   gain                Gain for weighting function
 * \param   threshold           Threshold for weighting function
 * \param   goal_direction      Goal direction for navigation
 */
typedef struct
{
    float pitch_;
    float gain_;
    float threshold_;
    float goal_direction_;
} saccade_controller_conf_t;


static inline saccade_controller_conf_t saccade_controller_default_config(void);



/**
 * \brief Saccade controller structure
 *
 * \param   flow_left           Left optic flow camera input
 * \param   flow_right;         Right optic flow camera input
 * \param   pitch               Pitch command for forward motion
 * \param   gain                Gain for weighting function
 * \param   threshold           Threshold for weighting function
 * \param   goal_direction      Goal direction for navigation
 * \param   azimuth             Table of azimuthal angles
 * \param   relative_nearness   Table of relative nearnesses
 * \param   attitude_command    Output for saccade

 */
class Saccade_controller
{
public:
    /**
     * \brief                Constructor
     *
     * \param   flow_left    Serial port for left optic flow camera
     * \param   flow_right   Serial port for right optic flow cameras
     * \param   config       Configuration structure
     */
    Saccade_controller(flow_t& flow_left, flow_t& flow_right, saccade_controller_conf_t config);


    /**
     * \brief   Init function
     *
     * \return  success
     */
    bool init(void);


    /**
     * \brief   Update of the saccade control
     *
     * \return  success
     */
    bool update(void);


    //Definition of the number of points used for the optic flow on each camera
    static const uint32_t N_points = 125;

    float                       pitch_;                             ///< Pitch command for forward motion
    float                       gain_;                              ///< Gain for importance of CAN
    float                       threshold_;                         ///< Threshold for importance of CAN
    float                       goal_direction_;                    ///< Goal direction for drone
    float                       azimuth_ [2 * N_points];            ///< Table of azimuthal angles
    float                       relative_nearness_ [2 * N_points];  ///< Table of Relative nearness
    float                       can_;
    float                       cad_;
    attitude_command_t          attitude_command_;                   ///< Attitude command given by the necessary saccade

    flow_t&                       flow_left_;                          ///< Left optic flow camera output
    flow_t&                       flow_right_;                         ///< Right optic flow camera output

};



static inline saccade_controller_conf_t saccade_controller_default_config(void)
{
    saccade_controller_conf_t conf;

    conf.pitch_          = 0.0f;
    conf.gain_           = 1.0f;
    conf.threshold_      = 1.0f;
    conf.goal_direction_ = 0.0f;

    return conf;
};


#endif /* ALTITUDE_CONTROLLER_HPP_ */
