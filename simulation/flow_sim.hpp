/*******************************************************************************
 * Copyright (c) 2009-2015, MAV'RIC Development Team
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
 * \file flow_sim.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Simulated Optic Flow sensors
 *
 ******************************************************************************/

#ifndef FLOW_SIM_HPP_
#define FLOW_SIM_HPP_

#include "drivers/flow.hpp"
#include "util/raytracing.hpp"
#include "util/matrix.hpp"
#include "simulation/dynamic_model.hpp"

/**
 * \brief   Interface for Optic Flow sensors
 */
class  Flow_sim: public Flow
{
    static const uint32_t ray_count_ = 90;
public:
    Flow_sim(Dynamic_model& dynamic_model_, raytracing::World& world, float orientation_azimuth = 0.0f);

    bool update(void);

    // flow_data_t of;               ///< Optic flow vectors
    // uint8_t     of_count;         ///< Number of optic flow vectors
    // flow_data_t of_loc;           ///< Location of optic flow vectors
    // uint32_t    last_update_us;   ///< Last update time in microseconds

private:
    Dynamic_model&      dynamic_model_;
    raytracing::World&  world_;

    raytracing::Ray     rays_[ray_count_];
    Mat<2,3>            jacob_[ray_count_];
};

#endif /* FLOW_SIM_HPP_ */
