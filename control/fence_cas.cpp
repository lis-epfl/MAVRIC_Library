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
 * \file fence_cas.cpp
 *
 * \author MAV'RIC Team
 * \author Cyril Stuber
 *
 * \brief This module takes care of simulating a fence and avoiding it.
 *
 ******************************************************************************/


#include "fence_cas.hpp"
//#include "../../src/central_data.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
#include "util/coord_conventions.h"
#include "util/constants.h"
#include "util/vectors.h"
#include "util/quick_trig.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
bool Fence_CAS::detect_line(float A[3], float B[3],const float C[3], float V[3], float gamma, float I[3])
{
	float Cp[3]={0,0,0};
	float S[3]={0,0,0};
	float Vnorm[3]={0,0,0};
	float pCCp[3]={0,0,0};
	vectors_normalize(V,Vnorm);
	float dmin=0.5f;
	for(int i =0; i<3;i++)
	{
		Cp[i]= C[i] + Vnorm[i] *  SCP(V,V)/(2*this->a_max);
	}
	for(int i =0; i<3;i++)
	{
		pCCp[i]= Cp[i]-C[i];
	}
	float temp = pCCp[0];
	pCCp[0]=-pCCp[1];
	pCCp[1]=temp;

	for(int i =0; i<3;i++)
	{
		S[i]= C[i] + (this->r_pz+dmin)/quick_trig_tan(gamma) * pCCp[i];
	}
	/// S found, compute detection?

	/*
	E = B-A = ( Bx-Ax, By-Ay )
	F = D-C = ( Dx-Cx, Dy-Cy )
	P = ( -Ey, Ex )
	h = ( (A-C) * P ) / ( F * P )
	This h number is the key. If h is between 0 and 1, the lines intersect, otherwise they don't. If F*P is zero, of course you cannot make the calculation, but in this case the lines are parallel and therefore only intersect in the obvious cases.

	The exact point of intersection is C + F*h
	*/
	float E[3]={0,0,0};
	for(int i =0; i<3;i++)
	{
		E[i]= S[i] - C[i];
	}

	float F[3]={0,0,0};
	for(int i =0; i<3;i++)
	{
		F[i]= B[i] - A[i];
	}

	float P[3]={0,0,0};
	P[0]=-E[1];
	P[1]=E[0];
	P[2]=E[2];

	float h= ((C[0]-A[0])*P[0]+(C[1]-A[1])*P[1])/(F[0]*P[0]+F[1]*P[1]);


	for(int i =0; i<3;i++)
	{
		I[i]= A[i] + h * F[i];
	}
	if (h>1)
	{
		return false;
	}
	else
	{
		return true;
	}

}
//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Fence_CAS::Fence_CAS(mavlink_waypoint_handler_t* waypoint_handler, position_estimation_t* postion_estimation)
:	sensor_res(3),
	a_max(1),
	r_pz(0),
	discomfort(0),
	waypoint_handler(waypoint_handler),
	pos_est(postion_estimation),
	detected_point({0,0,0}),
	fov(60.0)
{

}
Fence_CAS::~Fence_CAS(void)
{
	//destructeur
}
//call with tasks
bool Fence_CAS::update(void)
{
	//use waypoint_handler->fence_list[i];
	//and waypoint_handler->number_of_fence_points;

	//print_util_dbg_print("TEST UPDATE");
	//recupère les variables central_data
	//pour chaque fence
		//pour chaque doublet de points
			//create_edge
			//calcul SI l uav detecte la fence
				//si oui
					//calcul des repulsions
					//mélange les repulsiosn avec la vitesse

	//injecte la nouevlle vitesse dans central-data

	for (int i=0; i < waypoint_handler->number_of_fence_points; i++)
	{
		int j=0;
		if (i == waypoint_handler->number_of_fence_points - 1)
		{
			j=0;
		}
		else
		{
			j=i+1;
		}
		float A[3]={0,0,0};
		float B[3]={0,0,0};
		float V[3]={0,0,0};
		float gamma=0.0;
		A[0]=waypoint_handler->fence_list[i].x;
		A[1]=waypoint_handler->fence_list[i].y;
		A[2]=waypoint_handler->fence_list[i].z;
		B[0]=waypoint_handler->fence_list[j].x;
		B[1]=waypoint_handler->fence_list[j].y;
		B[2]=waypoint_handler->fence_list[j].z;
		float I[3]={0,0,0};
		//position_estimation_get_semilocal_velocity
		detect_line(A,B,pos_est->last_gps_pos.pos,V, gamma,I);
		//waypoint_struct_t; .x .y .z
		//pos_est->local_position.pos[x,y,z]
		//pos_est->last_gps_pos.pos[x,y,z]
		//fence points:
		//waypoint_handler->fence_list[i];// .x .y .z
		//waypoint_handler->fence_list[j];// .x .y .z
		j++;

	}

	return true;
}
void Fence_CAS::add_fence(void)
{
	//add a fence to the cas
}
void Fence_CAS::del_fence(uint8_t fence_id)
{
	//del a fence to the cas
}
void Fence_CAS::set_a_max(void)
{
	//add a fence to the cas
}
void Fence_CAS::get_a_max(void)
{
	//add a fence to the cas
}
void Fence_CAS::set_r_pz(void)
{
	//add a fence to the cas
}
void Fence_CAS::get_r_pz(void)
{
	//add a fence to the cas
}
void Fence_CAS::get_disconfort(void)
{
	//add a fence to the cas
}
void Fence_CAS::set_disconfort(void)
{
	//add a fence to the cas
}
void Fence_CAS::get_sensor_res(void)
{
	//add a fence to the cas
}
void Fence_CAS::set_sensor_res(void)
{
	//add a fence to the cas
}


// to use:

//position_estimation_t
/*float vel_bf[3];                        ///< 3D velocity in body frame
    float vel[3];                           ///< 3D velocity in global frame

    float last_alt;                         ///< Value of the last altitude estimation
    float last_vel[3];                      ///< Last 3D velocity

    local_position_t local_position;        ///< Local position
    local_position_t last_gps_pos;          ///< Coordinates of the last GPS position

    bool fence_set; */
