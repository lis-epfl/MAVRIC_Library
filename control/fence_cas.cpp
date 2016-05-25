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
#include "util/maths.h"

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
Fence_CAS::Fence_CAS(Mavlink_waypoint_handler* waypoint_handler, Position_estimation* postion_estimation, control_command_t* controls)
:	a_max(1),
	r_pz(0.5),
	comfort(0.5),
	waypoint_handler(waypoint_handler),
	pos_est(postion_estimation),
	controls(controls),
	tahead(2.0),
//	repulsion({0,0,0}),
	coef_roll(0.6),
	maxsens(10.0),
	maxradius(15.0),
	count(0)
{

}
Fence_CAS::~Fence_CAS(void)
{
	// Eraser
}
bool Fence_CAS::update(void)
{
	// Initializaion of variables
	static float old_distAC[MAX_WAYPOINTS];					// Table of the old distance to each fencepoint (used for small angles)
	bool angle_detected=false;									// Flag to reset the ROLL command

	for (int k=0;k<3;k++)									// Reset the repulsion command
	{
		this->repulsion[k]=0.0;
	}
	float C[3]={pos_est->local_position.pos[0],pos_est->local_position.pos[1],pos_est->local_position.pos[2]};	// Position of the Quad
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
	max_ang = 0.25*PI;
	this->coef_roll=1.0f;
	this->tahead=0.0f;
	int interp_type = 2;						// Define the interpolation type of the repulsion

	for(int i =0; i<3;i++)
	{
		S[i]= C[i] + Vnorm[i] * (this->r_pz/*protection zone*/ +SCP(V,V)/(2*this->a_max)/*dstop*/ + dmin/*dmin*/ + this->tahead * Vval /*d_ahead*/);
	}

	this->count++;
	if(this->count==100)
	{
		print_util_dbg_print("");print_util_dbg_putfloat(C[1],5);
		print_util_dbg_print(",");print_util_dbg_putfloat(C[0],5);
		print_util_dbg_print("\n");
		this->count=0;
	}
	/*FOR EACH FENCE*/
	for(int n=0;n<MAX_OUTFENCE+1;n++)
	{
		uint16_t nbFencePoints = *waypoint_handler->all_fence_points[n];
		Mavlink_waypoint_handler::waypoint_struct_t* CurFence_list  = this->waypoint_handler->all_fences[n];
		float* CurAngle_list = waypoint_handler->all_fence_angles[n];

		float dist[nbFencePoints];	// Table of distance to each fence
		float fencerep = 0.0f;		// Repulsion from fences
		float pointrep = 0.0f;		// Repulsion from angles


		for (int i=0; i < nbFencePoints; i++) 	// loop through all pair of fence points
		{
			/*INIT*/
			int j=0;
			if (i == nbFencePoints - 1)
			{
				j=0;
			}
			else
			{
				j=i+1;
			}
			// First point A, second point B
			global_position_t Agpoint = {CurFence_list[i].y, CurFence_list[i].x,(float)CurFence_list[i].z, 0.0f};
			global_position_t Bgpoint = {CurFence_list[j].y, CurFence_list[j].x,(float)CurFence_list[j].z, 0.0f};
			local_position_t Alpoint = coord_conventions_global_to_local_position(Agpoint,this->pos_est->local_position.origin);
			local_position_t Blpoint = coord_conventions_global_to_local_position(Bgpoint,this->pos_est->local_position.origin);

			float A[3]={Alpoint.pos[0],Alpoint.pos[1],Alpoint.pos[2]};
			float B[3]={Blpoint.pos[0],Blpoint.pos[1],Blpoint.pos[2]};

			// Only 2D detection:
			A[2]=C[2];
			B[2]=C[2];
			/*END INIT*/

			/*Fencepoint repulsion*/
			/*Min radius method*/

			float AB[3]={B[0]-A[0],B[1]-A[1],0.0};
			float pAB[3]={-AB[1],AB[0],0.0};
			vectors_normalize(AB,AB);
			vectors_normalize(pAB,pAB);
			float M[3]={0,0,0};
			this->maxradius = 5; //inner angle circle radius

			float distAS = detect_seg(A,A,C,S,V,I,J);	// Compute distance from drone to fencepoint.

			for(int k=0;k<3;k++)
			{
				M[k] = A[k] + (this->maxradius+this->maxsens)*(AB[k]/quick_trig_tan(CurAngle_list[i]/2.0) + pAB[k]);
			}

			float MS[3] = {S[0]-M[0],S[1]-M[1],0.0};
			float distMC=detect_seg(M,M,C,S,V,I,J);

			float MA[3] = {A[0]-M[0],A[1]-M[1],0.0};
			float distMA=vectors_norm(MA);


			if((distAS <= (distMA))&&(distMC >= this->maxradius)&&(angle_detected==false))
			{
				float aratio=(distMC - this->maxradius)/this->maxsens;	// Compute ratio for interpolation, ratio is only for the first maxsens, then saturates at 1
				float rep[3]={MS[0],MS[1],0.0};					// Repulsion local frame
				gftobftransform(C, S, rep);								// Repulsion body frame

//				vectors_normalize(rep,rep);
				rep[1]=(rep[1]>=0?-1:1) ;// Extract repulsion direction in body frame
				pointrep += -rep[1]*this->coef_roll*max_ang*interpolate(aratio,interp_type); // Add repulsion
				angle_detected=true;
//				print_util_dbg_print("||Arep||");print_util_dbg_putfloat(i+1,0);
//				print_util_dbg_print("||angle||");print_util_dbg_putfloat(CurAngle_list[i]*180/PI,5);
//				print_util_dbg_print("|rep|");print_util_dbg_putfloat(pointrep,5);
//				print_util_dbg_print("|interpval|");print_util_dbg_putfloat(interpolate(aratio,interp_type),5);

//				print_util_dbg_print("|interp|");print_util_dbg_putfloat(interpolate(aratio,interp_type),5);
//				print_util_dbg_print("|oldAC|");print_util_dbg_putfloat(old_distAC[i],15);
//				print_util_dbg_print("\n");
			}
			else
			{
				;
			}
			/*END Min radius method*/

			/*Fence repulsion*/
			dist[i] = detect_seg(A,B,C,S,V,I,J);

			if((dist[i] < this->maxsens)&&(angle_detected==false))
			{
				float rep[3]={A[1]-B[1],B[0]-A[0],0.0};						// Repulsion local frame
				gftobftransform(C, S, rep);									// Repulsion body frame
//				vectors_normalize(rep,rep);
				rep[1]=(rep[1]>=0?1:-1); 									// Extract repulsion direction in body frame, 1 = clockwise / -1 = counterclockwise

				float fratio = dist[i]/this->maxsens;						// Compute ratio for interpolation
				fencerep +=-rep[1]*this->coef_roll*max_ang*interpolate(fratio,interp_type);

//				print_util_dbg_print("||Frep||");print_util_dbg_putfloat(i+1,0);
//				print_util_dbg_print("||");print_util_dbg_putfloat(fencerep,5);
//				print_util_dbg_print("||");print_util_dbg_putfloat(this->repulsion[1],5);
//				print_util_dbg_print("|ratio|");print_util_dbg_putfloat(interpolate(fratio,interp_type),5);
//				print_util_dbg_print("\n");
			}
			else
			{
				;
			}
			/*END Fence repulsion*/
		}
		if(angle_detected==true)
		{
			angle_detected=false;
			this->repulsion[1] += pointrep;
		}
		else
		{
			this->repulsion[1] += fencerep;
		}
	}

	/*END FOR EACH FENCE*/
	// Clip the repulsion
	if(this->repulsion[1]>max_ang)
	{
		this->repulsion[1]=max_ang;
	}
	if(this->repulsion[1]<-max_ang)
	{
		this->repulsion[1]=-max_ang;
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
