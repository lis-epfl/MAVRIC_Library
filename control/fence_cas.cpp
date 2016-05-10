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
// #include "../../src/central_data.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
#include "util/coord_conventions.h"
#include "util/constants.h"
#include "util/vectors.h"
#include "util/quick_trig.h"
#include "util/vectors.h"

#define SMALL_NUM 0.000001
#define CLOSE_NUM 0.1
}


// ------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
// ------------------------------------------------------------------------------
float Fence_CAS::detect_seg(float A[3], float B[3], float C[3], float S[3] , float V[3], float I[3],float J[3])
{
	// Taken and adapted from http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect, consulted on april 2016
	float	u[3] = {S[0]-C[0],S[1]-C[1],S[2]-C[2]};		// Quad segment
	float	v[3] = {B[0]-A[0],B[1]-A[1],B[2]-A[2]};		// Fence segment
	float	w[3] = {C[0]-A[0],C[1]-A[1],C[2]-A[2]}; 	// Initial distance segment

	float  a = vectors_scalar_product(u,u);     	// Always >= 0
	float  b = vectors_scalar_product(u,v);
	float  c = vectors_scalar_product(v,v);     	// Always >= 0
	float  d = vectors_scalar_product(u,w);
	float  e = vectors_scalar_product(v,w);

	float  D = a*c - b*b;    	// Always >= 0
	float  sc, sN, sD = D;    	// sc = sN / sD, default sD = D >= 0
	float  tc, tN, tD = D;    	// tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	if (D < SMALL_NUM) { 	// the lines are almost parallel Value is defined to be small enough
		sN = 0.0;     		// force using point P0 on segment S1
		sD = 1.0;     		// to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	}
	else {         			// get the closest points on the infinite lines
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < 0.0) {    	// sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD) {	// sc > 1 => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}
	if (tN < 0.0) {      	// tc < 0 => the t=0 edge is visible
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD) {   	// tc > 1 => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d + b);
			sD = a;
		}
	}
	// finally do the division to get sc and tc
	sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
	tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);
	float dp[3]={0,0,0}; 	// dp = distance vector between I and J, dp = J-I

	for (int i=0; i<3;i++)
	{
		I[i] = A[i]+tc*v[i];
		J[i] = C[i] + sc*u[i];
		dp[i]=w[i]+sc*u[i] - tc*v[i];
	}
	// Only 2D detection of segments
	I[2]=C[2];
	J[2]=C[2];
	// dp[2]=0; // only 2D
	// return the closest distance
	return vectors_norm(dp);
}

// ------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
// ------------------------------------------------------------------------------
Fence_CAS::Fence_CAS(mavlink_waypoint_handler_t* waypoint_handler, position_estimation_t* postion_estimation, control_command_t* controls)
:	a_max(1),
	r_pz(1),
	comfort(0.845),
	waypoint_handler(waypoint_handler),
	pos_est(postion_estimation),
	controls(controls),
	repulsion({0,0,0}),
	tahead(2.0),
	coef_roll(0.01),
	maxsens(10.0)
{

}
Fence_CAS::~Fence_CAS(void)
{
	// Eraser
}
bool Fence_CAS::update(void)
{
	// Initializaion of variables
	float dist[waypoint_handler->number_of_fence_points];	// Table of distance to each fence
	static float old_distAC[MAX_WAYPOINTS];					// Table of the old distance to each fencepoint (used for small angles)
	bool detected=false;									// Flag to reset the ROLL command
	for (int k=0;k<3;k++)									// Reset the repulsion command
	{
		this->repulsion[k]=0.0;
	}
	float C[3]={this->pos_est->last_gps_pos.pos[0],this->pos_est->last_gps_pos.pos[1],this->pos_est->last_gps_pos.pos[2]};	// Position of the Quad
	float S[3]={0,0,0};																										// Position of the heading

	float I[3]={0,0,0};	// Detected point on fence segment
	float J[3]={0,0,0}; // Detected point on quad segment

	float V[3]={0,0,0};	// Velocity in local frame
	for (int k=0;k<3;k++)
	{
		V[k]=pos_est->vel[k];
	}

	float Vnorm[3]={0,0,0};
	vectors_normalize(V,Vnorm);
	float Vval = vectors_norm(V);

	float dmin=2*this->r_pz; 					// Safe zone around the drone ,can be adjusted
	float max_ang= 0.25*PI*(1-this->comfort);	// Adjusts the maximal angle in function of the comfort
	int interp_type = 2;						// Define the interpolation type of the repulsion

	for(int i =0; i<3;i++)
	{
		S[i]= C[i] + Vnorm[i] * (this->r_pz/*protection zone*/ +SCP(V,V)/(2*this->a_max)/*dstop*/ + dmin/*dmin*/ + this->tahead * Vval /*d_ahead*/);
	}

	for (int i=0; i < waypoint_handler->number_of_fence_points; i++) // loop through all pair of fence points
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
		// First point A, second point B
		global_position_t Agpoint = {this->waypoint_handler->fence_list[i].y, this->waypoint_handler->fence_list[i].x,(float)this->waypoint_handler->fence_list[i].z, 0.0f};
		global_position_t Bgpoint = {this->waypoint_handler->fence_list[j].y, this->waypoint_handler->fence_list[j].x,(float)this->waypoint_handler->fence_list[j].z, 0.0f};
		local_position_t Alpoint = coord_conventions_global_to_local_position(Agpoint,this->pos_est->local_position.origin);
		local_position_t Blpoint = coord_conventions_global_to_local_position(Bgpoint,this->pos_est->local_position.origin);

		float A[3]={Alpoint.pos[0],Alpoint.pos[1],Alpoint.pos[2]};
		float B[3]={Blpoint.pos[0],Blpoint.pos[1],Blpoint.pos[2]};
		// Only 2D detection:
		A[2]=C[2];
		B[2]=C[2];

		/*Fencepoint repulsion*/
		float angle_rep_radius = this->maxsens;
		if(waypoint_handler->fance_angle_list[i]>PI/2.0) // Check if the angle is smaller than pi/2
		{

		}
		else	// The angle is smaller than pi/2
		{
			float cos = quick_trig_cos(waypoint_handler->fance_angle_list[i]/2.0)*(1 - waypoint_handler->fance_angle_list[i]/PI)*2;
			angle_rep_radius = 4*cos*cos*cos*this->maxsens; // The smaller the angle, the bigger the repulsion radius
			float D[3]={0,0,0}; // D is the precedent point (current A, next B, precedent D)
			if(i==0)
			{
				global_position_t Dgpoint = {this->waypoint_handler->fence_list[waypoint_handler->number_of_fence_points-1].y, this->waypoint_handler->fence_list[waypoint_handler->number_of_fence_points-1].x,(float)this->waypoint_handler->fence_list[waypoint_handler->number_of_fence_points-1].z, 0.0f};
				local_position_t Dlpoint = coord_conventions_global_to_local_position(Dgpoint,this->pos_est->local_position.origin);
				D[0]=Dlpoint.pos[0];D[1]=Dlpoint.pos[1];D[2]=Dlpoint.pos[2];
			}
			else
			{
				global_position_t Dgpoint = {this->waypoint_handler->fence_list[i-1].y, this->waypoint_handler->fence_list[i-1].x,(float)this->waypoint_handler->fence_list[i-1].z, 0.0f};
				local_position_t Dlpoint = coord_conventions_global_to_local_position(Dgpoint,this->pos_est->local_position.origin);
				D[0]=Dlpoint.pos[0];D[1]=Dlpoint.pos[1];D[2]=Dlpoint.pos[2];
			}
			float fence2[3]={D[0]-A[0],D[1]-A[1],D[2]-A[2]};	// Compute vector AD
			float fence1[3]={B[0]-A[0],B[1]-A[1],B[2]-A[2]};	// Compute vector AB
			// Clip the repulsion radius with the length of the adjacent segments
			if(vectors_norm(fence1)<angle_rep_radius)
			{
				angle_rep_radius = vectors_norm(fence1);
			}
			if(vectors_norm(fence2)<angle_rep_radius)
			{
				angle_rep_radius = vectors_norm(fence2);
			}
		}

		float distAC = detect_seg(A,A,C,S,V,I,J);	// Compute distance from drone to fencepoint.
		if((distAC >= -(angle_rep_radius))&(distAC < angle_rep_radius))
		{
			if((old_distAC[i]>=distAC)) // If the drone is heading toward the fencepoint
			{
				float ratio=(angle_rep_radius-distAC)/this->maxsens;	// Compute ratio for interpolation, ratio is only for the first maxsens, then saturates at 1
				float rep[3]={A[0]-S[0],A[1]-S[1],0.0};					// Repulsion local frame
				gftobftransform(C, S, rep);								// Repulsion body frame
				vectors_normalize(rep,rep);
				rep[1]=(rep[1]>=0?-1:1) ;								// Extract repulsion direction in body frame
				this->repulsion[1]+=- rep[1]*this->coef_roll*max_ang*interpolate(ratio,interp_type); // Add repulsion
				detected=true;												// Enable detection flag
			}
		}
		old_distAC[i]=distAC;											// Store old distance to fencepoint

		/*Fence repulsion*/
		dist[i] = detect_seg(A,B,C,S,V,I,J);							// Compute distance to the fence
		if((dist[i] >= -(this->maxsens))&(dist[i] < this->maxsens))
		{
			float rep[3]={A[1]-B[1],B[0]-A[0],0.0};						// Repulsion local frame
			gftobftransform(C, S, rep);									// Repulsion body frame
			vectors_normalize(rep,rep);
			rep[1]=(rep[1]>=0?1:-1); 									// Extract repulsion direction in body frame, 1 = clockwise / -1 = counterclockwise

			float ratio = dist[i]/this->maxsens;						// Compute ratio for interpolation

// 			this->repulsion[0]+=0.0;
			this->repulsion[1]+=-rep[1]*this->coef_roll*max_ang*interpolate(ratio,interp_type);
// 			this->repulsion[2]+=0.0;
			detected=true;												// Enable detection flag
		}
		else
		{

		}

	}
	// Clip the repulsion
	if(this->repulsion[1]>max_ang)
	{
		this->repulsion[1]=max_ang;
	}
	if(this->repulsion[1]<-max_ang)
	{
		this->repulsion[1]=-max_ang;
	}
	if(detected==false)	// Reset the roll is nothing is detected
	{
		controls->rpy[ROLL] = 0.0;
	}

	return true;
}
float Fence_CAS::interpolate(float r, int type) // type=x, 0: linear, 1: cos, 2:cos2
{
	if(type==0) // Linear interpolation
	{
		if((r>0.0)&&(r<1.0))
		{
			return 1-r;
		}
		else
		{
			return 1;
		}
	}
	else if(type==1) // Cos interpolation
	{
		if((r>0.0)&&(r<1.0))
		{
			return 0.5*quick_trig_cos(r*PI)+0.5;
		}
		else
		{
			return 1;
		}
	}
	if(type==2) // Other cos interpolation
	{
		if((r>0.0)&&(r<1.0))
		{
			return quick_trig_cos(r*PI/2.0+PI/2.0)+1;
		}
		else
		{
			return 1;
		}
	}
	return 0.0;
}
void Fence_CAS::gftobftransform(float C[3], float S[3], float rep[3])
{
	float temp0 = (S[0]-C[0])*rep[0] + (S[1]-C[1])*rep[1];
	float temp1 = (S[1]-C[1])*rep[0] + (C[0]-S[0])*rep[1];
	rep[0]=temp0;
	rep[1]=temp1;
	rep[2]=0.0;
}
void Fence_CAS::set_a_max(void)
{
	// add a fence to the cas
}
void Fence_CAS::get_a_max(void)
{
	// add a fence to the cas
}
void Fence_CAS::set_r_pz(void)
{
	// add a fence to the cas
}
void Fence_CAS::get_r_pz(void)
{
	// add a fence to the cas
}
void Fence_CAS::get_disconfort(void)
{
	// add a fence to the cas
}
void Fence_CAS::set_disconfort(void)
{
	// add a fence to the cas
}
float Fence_CAS::get_repulsion(int axis)
{
	return this->repulsion[axis];
}
float Fence_CAS::get_max_angle(void)
{
	return 0.25*PI*(1-this->comfort);
}
