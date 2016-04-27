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
 * \file fence_cas.hpp
 *
 * \author MAV'RIC Team
 * \author Cyril Stuber
 *
 * \brief 	This module takes care of simulating a fence and avoiding it.
 * 			"cas" stand for Collision Avoiding System.
 *
 ******************************************************************************/


#ifndef FENCE_CAS_H_
#define FENCE_CAS_H_


//check if thoses are usefull
//#include "control/fence.hpp"
#include "communication/mavlink_waypoint_handler.hpp"
//#include "sensing/position_estimation.hpp"
#include "sensing/position_estimation.hpp"


extern "C"
{
#include "control/control_command.h"
}

class Fence_CAS
{
public:
	Fence_CAS(mavlink_waypoint_handler_t* waypoint_handler, position_estimation_t* postion_estimation,control_command_t* controls );
	~Fence_CAS(void);
	bool update(void);
	void add_fence(void);
	void del_fence(uint8_t fence_id);

	void set_sensor_res(void);
	void get_sensor_res(void);
	void set_disconfort(void);
	void get_disconfort(void);
	void set_a_max(void);
	void get_a_max(void);
	void set_r_pz(void);
	void get_r_pz(void);
	float get_repulsion(int axis);
	void gftobftransform(float C[3], float S[3], float rep[3]);
	float interpolate(float r, int type);
	float 								maxsens;
	float								a_max; ///<maximal deceleration [m/s^2]
	float								r_pz; ///< radius of Protection Zone
	float								discomfort; ///<[0,1] intensity of the reaction
	float								tahead; ///<[0,1] intensity of the reaction
	float								coef_roll; ///<[0,1] intensity of the reaction




private:

	float detect_seg(float A[3], float B[3], float C[3], float S[3] , float V[3], float I[3],float J[3]);
	float detect_line(local_position_t A, local_position_t B,local_position_t C, float V[3], float gamma, float I[3]);
	uint8_t								sensor_res; ///< simulate sensor resolution, spatial resolution between two sensors. [deg]
	mavlink_waypoint_handler_t* 		waypoint_handler;
	const position_estimation_t*        pos_est;                    ///< Estimated position and speed (input)
	control_command_t* 					controls;
	//velocity_command_t&                 velocity_command;           ///< Velocity command (output)
	float 								detected_point[3];
	float 								repulsion[3];
};

#endif /*FENCE_CAS_H_*/
