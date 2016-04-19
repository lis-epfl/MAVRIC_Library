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
 * \file wing_model.hpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Jacquemin
 * \author Julien Lecoeur
 *
 * \brief Model of a wing
 *
 ******************************************************************************/


#ifndef WINGMODEL_HPP_
#define WINGMODEL_HPP_

extern "C"
{
#include "util/constants.h"
#include "util/quaternions.h"
}

/**
 * \brief Force structure
 */
typedef struct
{
   float torque[3];
   float force[3];
} wing_model_forces_t;

/**
 * \brief
 */
class Wing_model
{
public:
    /**
     * @brief   Constructor
     *
     * \param   flap_angle    angle of the flap in degrees
     * \param   orientation   quaternion of the orientation of the wing in bf
     * \param   x_position    x distance between wing COG and plane COG
     * \param   y_position    y distance between wing COG and plane COG
     * \param   z_position    z distance between wing COG and plane COG
     * \param   area          area of the wing in m^2
     * \param   chord         chord length of the wing
     */
    Wing_model(float flap_angle,
      quat_t orientation,
      float x_position_bf,
      float y_position_bf,
      float z_position_bf,
      float area,
      float chord);

     /**
     * \brief   Computes the forces applied on the wing
     * \detail  Takes the wind speed vector and computes the force that applies on the wing (3 forces and 3 moments are possible)
     *
     * \param	wind_bf	speed of wind relatively to the plane, in bf
     *
     * \return  The 3 forces and 3 moments that applies on the wing
     */
    wing_model_forces_t compute_forces(float wind_bf[3]);

    /**
    * \brief    Allows to change the flap angle
    *
    * \param    angle   the new angle, in degrees
    */
    void set_flap_angle(float angle);

private:
    float flap_angle_;        // angle of the flap
    quat_t orientation_;      // orientation of the wing in bf reference
    float position_bf_[3];    // position of the wing COG from the plane COG in bf
    float area_;              // surface of the wing (m^2)
    float chord_;             // length of the chord (m)
    float lookup_Cl_[181];    // Lookup table of the Cl coefficient
    float lookup_Cd_[181];    // Lookup table of the Cd coefficient
    float lookup_Cm_[181];    // Lookup table of the Cm coefficient

    /**
    * \brief    Initializes the 3 lookup tables
    */
    void init_lookup();

    /**
    * \brief    Get Cl coefficient
    *
    * \Return   value of Cl
    */
    float get_cl(float aoa, float flap_angle);

    /**
    * \brief    Get Cd coefficient
    *
    * \Return   value of Cd
    */
    float get_cd(float aoa, float flap_angle);

    /**
    * \brief    Get Cm coefficient
    *
    * \Return   value of Cm
    */
    float get_cm(float aoa, float flap_angle);

    /**
    * \brief    Transform the forces from wing reference frame to bf
    *
    * \param    forces_wf   the forces in the wing frame
    *
    * \Return   forces in the body frame
    */
    wing_model_forces_t forces_wing_to_bf(wing_model_forces_t forces_wf);
};

#endif /* WING_MODEL_HPP_ */
