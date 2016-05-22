/*******************************************************************************
 * Copyfront (c) 2009-2016, MAV'RIC Development Team
 * All fronts reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyfront notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyfront notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyfront holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYFRONT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYFRONT HOLDER OR CONTRIBUTORS BE
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
#include "util/quick_trig.h"
#include "sensing/ahrs.h"
#include "control/pid_controller.h"
#include "sensing/altitude.h"
}
#include "sensing/position_estimation.hpp"

#include "drivers/flow.hpp"

#include "util/buffer.hpp"
#include "hal/common/time_keeper.hpp"

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
    pid_controller_conf_t pid_config;
} saccade_controller_conf_t;


static inline saccade_controller_conf_t saccade_controller_default_config(void);



/**
 * \brief Saccade controller structure
 *
 * \param   flow_back           back optic flow camera input
 * \param   flow_front;         Front optic flow camera input
 * \param   pitch               Pitch command for forward motion
 * \param   gain                Gain for weighting function
 * \param   threshold           Threshold for weighting function
 * \param   goal_direction      Goal direction for navigation
 * \param   azimuth             Table of azimuthal angles
 * \param   relative_nearness   Table of relative nearnesses
 * \param   attitude_command    Output for saccade

 */

 typedef enum
{
    PRESACCADE,         ///< presaccade state
    SACCADE,            ///< Saccade state
    INTERSACCADE,        ///< intersaccade state

} saccade_state_t;


class Saccade_controller
{
public:
    /**
     * \brief                Constructor
     *
     * \param   flow_back    Serial port for back optic flow camera
     * \param   flow_front   Serial port for front optic flow cameras
     * \param   ahrs         Attitude and heading reference system
     * \param   config       Configuration structure
     */
    Saccade_controller(Flow& flow_front, Flow& flow_back, const ahrs_t& ahrs,
                       Position_estimation& position_estimation,
                       saccade_controller_conf_t config = saccade_controller_default_config());


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
    // static const uint32_t N_points = 125;
    static const uint32_t N_points = 75;

    float                       pitch_;                             ///< Pitch command for forward motion
    float                       gain_;                              ///< Gain for importance of CAN
    float                       threshold_;                         ///< Threshold for importance of CAN
    float                       goal_direction_;                    ///< Goal direction for drone
    float                       azimuth_ [2 * N_points];            ///< Table of azimuthal angles
    float                       derotated_flow_front_ [N_points];   ///< Table of Relative nearness
    float                       derotated_flow_back_ [N_points];    ///< Table of Relative nearness
    float                       relative_nearness_ [2 * N_points];  ///< Table of Relative nearness
    float                       inv_sin_azimuth_ [2 * N_points];
    float                       cos_azimuth_[2 * N_points];
    float                       sin_azimuth_[2 * N_points];
    float                       can_;
    float                       cad_;
    float                       intersaccade_time_;
    float                       weighted_function_;
    float                       derotation_constant_;
    float                       last_derotation_yaw_velocity_;

    uint64_t                    last_saccade_;

    pid_controller_t            altitude_pid_;


    attitude_command_t          attitude_command_;                   ///< Attitude command given by the necessary saccade

    Flow&                       flow_front_;                         ///< Front optic flow camera output
    Flow&                       flow_back_;                          ///< back optic flow camera output

    const ahrs_t&               ahrs_;                               ///< Attitude and heading reference system

    float                       altitude_value_;

    velocity_command_t          velocity_command_;

    Position_estimation&        position_estimation_;

    bool                        is_time_initialized_;               ///< Flag for time of presaccadic state

    saccade_state_t             saccade_state_;                     ///< Saccade modes (intersaccade, saccade, presaccade)

    Buffer_tpl<3,float>         yaw_velocity_buffer_;
};



static inline saccade_controller_conf_t saccade_controller_default_config(void)
{


    saccade_controller_conf_t conf;

    conf.pid_config                         = {};
    conf.pid_config.p_gain                  = 0.2f;
    conf.pid_config.clip_min                = -0.5f;
    conf.pid_config.clip_max                = 0.5f;
    conf.pid_config.integrator              = {};
    conf.pid_config.integrator.gain         = 0.0f;
    conf.pid_config.integrator.accumulator  = 0.0f;
    conf.pid_config.integrator.clip_pre     = 0.0001f;
    conf.pid_config.integrator.clip         = 0.5f;
    conf.pid_config.differentiator          = {};
    conf.pid_config.differentiator.gain     = 0.0f;
    conf.pid_config.differentiator.previous = 0.0f;
    conf.pid_config.differentiator.clip     = 0.65f;
    conf.pid_config.soft_zone_width         = 0.0f;


    conf.pitch_          = 0.0f;
    conf.gain_           = 1.0f;
    conf.threshold_      = 1.0f;
    conf.goal_direction_ = 1.5f;

    return conf;
};

/**
 * \brief  Glue method for scheduler
 */
static inline bool task_saccade_controller_update(Saccade_controller* saccade_controller)
{
    return saccade_controller->update();
};

#endif /* ALTITUDE_CONTROLLER_HPP_ */
