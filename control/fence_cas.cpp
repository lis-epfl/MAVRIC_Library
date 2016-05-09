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
#include "util/vectors.h"

#define SMALL_NUM 0.000001
#define CLOSE_NUM 0.1
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
float Fence_CAS::detect_line(local_position_t Al, local_position_t Bl,local_position_t Cl, float V[3], float gamma, float I[3])
{

	float A[3]={Al.pos[0],Al.pos[1],Al.pos[2]};
	float B[3]={Bl.pos[0],Bl.pos[1],Bl.pos[2]};
	float C[3]={Cl.pos[0],Cl.pos[1],Cl.pos[2]};

	float Cp[3]={0,0,0};
	float CCp[3]={0,0,0};
	float S[3]={0,0,0};
	float pCCp[3]={0,0,0}; //perpendicular to CCp
	float Vnorm[3]={0,0,0};
	vectors_normalize(V,Vnorm);
	float dmin=0.5f;

	for(int i =0; i<3;i++)
	{
		I[i]=0;
		Cp[i]= C[i] + Vnorm[i] *  SCP(V,V)/(2*this->a_max);
	}
	for(int i =0; i<3;i++)
	{
		CCp[i]= Cp[i]-C[i];

	}
	pCCp[0]=CCp[1];
	pCCp[1]=CCp[0];
	pCCp[2]=CCp[2];

	for(int i =0; i<3;i++)
	{
		if(gamma==0)
		{
			/*Tahaed =  0.1*/
			S[i]= C[i] + Vnorm[i]*0.1* CCp[i];
		}

		else
		{
			S[i]= C[i] + (this->r_pz+dmin)/quick_trig_tan(gamma) * CCp[i]+(this->r_pz+dmin)*pCCp[i];
		}
	}
	//The exact point of intersection is C + F*h

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

	float P[3]={0,0,0}; //P stands here for perpendicular
	P[0]=-E[1];
	P[1]=E[0];
	P[2]=E[2];

	//h is the distance from A to B of which point is detected
	float h= ((C[0]-A[0])*P[0]+(C[1]-A[1])*P[1])/(F[0]*P[0]+F[1]*P[1]);
	float dist=0;
	if ((h>1.0)|(h<0.0))
	{
		print_util_dbg_print("NO_Ipoint||");
	}


	else
	{

		for(int i =0; i<3;i++) // I[3] is the intersection point found
		{
			I[i]= A[i] + h * F[i];

			if(i==2)
			{
				I[2]=C[2];
			}
			this->detected_point[i]=I[i];
			dist+= (C[i]-I[i])*(C[i]-I[i]);
		}

		float maxsens= SCP(V,V)/(2*this->a_max);
		if(dist>=maxsens*maxsens)
		{
			print_util_dbg_print("TOO FAR((");
			print_util_dbg_putfloat(maxsens*maxsens,2);
			print_util_dbg_print("))");
		}
		I[2] = maxsens*maxsens;

	}
	return dist;

}

float Fence_CAS::detect_seg(float A[3], float B[3], float C[3], float S[3] , float V[3], float I[3],float J[3])
{

	float	u[3] = {S[0]-C[0],S[1]-C[1],S[2]-C[2]};	//quad segment
	float	v[3] = {B[0]-A[0],B[1]-A[1],B[2]-A[2]};	//fence segment
	float	w[3] = {C[0]-A[0],C[1]-A[1],C[2]-A[2]}; //initial distance segment

	float    a = vectors_scalar_product(u,u);         // always >= 0
	float    b = vectors_scalar_product(u,v);
	float    c = vectors_scalar_product(v,v);         // always >= 0
	float    d = vectors_scalar_product(u,w);
	float    e = vectors_scalar_product(v,w);

	float    D = a*c - b*b;        // always >= 0
	float    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
	float    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	if (D < SMALL_NUM) { // the lines are almost parallel Value is defined to be small enough
		sN = 0.0;         // force using point P0 on segment S1
		sD = 1.0;         // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	}
	else {                 // get the closest points on the infinite lines
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}
	if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
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
	else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d +  b);
			sD = a;
		}
	}
	// finally do the division to get sc and tc
	sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
	tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);
	float dp[3]={0,0,0}; //dp= distance vecto between I and J, dp = J-I

	for (int i=0; i<3;i++)
	{
		I[i] = A[i]+tc*v[i];
		J[i] = C[i] + sc*u[i];
		dp[i]=w[i]+sc*u[i] - tc*v[i];
	}
	//ONLY 2D detection of segments
	I[2]=C[2];
	J[2]=C[2];
//	dp[2]=0; //only 2D
//	return (sc>0.001?vectors_norm(dp):-vectors_norm(dp));   // return the closest distance
	return vectors_norm(dp);
}



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Fence_CAS::Fence_CAS(mavlink_waypoint_handler_t* waypoint_handler, position_estimation_t* postion_estimation, control_command_t* controls)
:	sensor_res(3),
	a_max(1),
	r_pz(0),
	comfort(0.5),
	waypoint_handler(waypoint_handler),
	pos_est(postion_estimation),
	controls(controls),
	detected_point({0,0,0}),
	repulsion({0,0,0}),
	tahead(2.0),
	coef_roll(0.005),
	maxsens(10.0)
{

}
Fence_CAS::~Fence_CAS(void)
{
	//destructeur
}
//call with tasks
bool Fence_CAS::update(void)
{
	//initializaion of variables
	float dist[waypoint_handler->number_of_fence_points];
	static float old_distAC[MAX_WAYPOINTS];
	int detected=0.0;
	for (int k=0;k<3;k++)
	{
		this->repulsion[k]=0.0;
	}
	float C[3]={this->pos_est->last_gps_pos.pos[0],this->pos_est->last_gps_pos.pos[1],this->pos_est->last_gps_pos.pos[2]};
	float S[3]={0,0,0};

	float I[3]={0,0,0}; //Detected point on fence segment
	float J[3]={0,0,0}; //Detected point on quad segment

	float V[3]={0,0,0};

	for (int k=0;k<3;k++)
	{
		V[k]=pos_est->vel[k];
	}

	float Vnorm[3]={0,0,0};
	vectors_normalize(V,Vnorm);
	float Vval = vectors_norm(V);

	float dmin=2*this->r_pz; //can be adjusted
	float max_ang= 0.25*PI*(1-this->comfort);
	int interp_type = 2;

	for(int i =0; i<3;i++)
	{
		S[i]= C[i] + Vnorm[i] *  (this->r_pz/*protection zone*/ +SCP(V,V)/(2*this->a_max)/*dstop*/ + dmin/*dmin*/ + this->tahead * Vval /*d_ahead*/);
	}

	for (int i=0; i < waypoint_handler->number_of_fence_points; i++) //loop through all pair of fence points
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
		global_position_t Agpoint = {this->waypoint_handler->fence_list[i].y, this->waypoint_handler->fence_list[i].x,(float)this->waypoint_handler->fence_list[i].z, 0.0f};
		global_position_t Bgpoint = {this->waypoint_handler->fence_list[j].y, this->waypoint_handler->fence_list[j].x,(float)this->waypoint_handler->fence_list[j].z, 0.0f};
		local_position_t Alpoint = coord_conventions_global_to_local_position(Agpoint,this->pos_est->local_position.origin);
		local_position_t Blpoint = coord_conventions_global_to_local_position(Bgpoint,this->pos_est->local_position.origin);

		float A[3]={Alpoint.pos[0],Alpoint.pos[1],Alpoint.pos[2]};
		float B[3]={Blpoint.pos[0],Blpoint.pos[1],Blpoint.pos[2]};
		//ONLY 2D detection:
		A[2]=C[2];
		B[2]=C[2];

		float angle_rep_radius = this->maxsens;
		//fence points repulsion
		if(waypoint_handler->fance_angle_list[i]>PI/2.0)
		{

		}
		else
		{
			float cos = quick_trig_cos(waypoint_handler->fance_angle_list[i]/2.0)*(1 - waypoint_handler->fance_angle_list[i]/PI)*2;
			angle_rep_radius = 4*cos*cos*cos*this->maxsens;
			float D[3]={0,0,0};
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
			float fence2[3]={D[0]-A[0],D[1]-A[1],D[2]-A[2]};//D[3]={Dlpoint.pos[0],Dlpoint.pos[1],Dlpoint.pos[2]};
			float fence1[3]={B[0]-A[0],B[1]-A[1],B[2]-A[2]};
			//test if angle_rep_radius isn't to big and influence other fences
			if(vectors_norm(fence1)<angle_rep_radius)
			{
				angle_rep_radius = vectors_norm(fence1);
			}
			if(vectors_norm(fence2)<angle_rep_radius)
			{
				angle_rep_radius = vectors_norm(fence2);
			}
		}
		float AC[3]={C[0]-A[0],C[1]-A[1],C[2]};
		float distAC = vectors_norm(AC);
		distAC = detect_seg(A,A,C,S,V,I,J);

//		if(i==3)
//		{
//			print_util_dbg_print("||");print_util_dbg_putfloat(i+1,0);
//			print_util_dbg_print("|dist|");print_util_dbg_putfloat(distAC,5);
//			print_util_dbg_print("|angle_rep|");print_util_dbg_putfloat(angle_rep_radius,5);
//			print_util_dbg_print("|angle|");print_util_dbg_putfloat(waypoint_handler->fance_angle_list[i]*MATH_RAD_TO_DEG,5);
//			print_util_dbg_print("\n");
//		}

		//use the distance to the points and not to the fences.
		//fence point repulsion
		if((distAC >= -(angle_rep_radius))&(distAC < angle_rep_radius))
		{
//			if(angle_rep_radius==0)
			if((old_distAC[i]>=distAC))
			{
				float ratio=(angle_rep_radius-distAC)/this->maxsens;
				float rep[3]={A[0]-S[0],A[1]-S[1],0.0};
				gftobftransform(C, S, rep);
				vectors_normalize(rep,rep);
				rep[1]=(rep[1]>=0?-1:1) ;
				this->repulsion[1]+=- rep[1]*this->coef_roll*max_ang*interpolate(ratio,interp_type); //sens of repulsion fixed
				detected++;
//				print_util_dbg_print("|REPULSION|");print_util_dbg_putfloat(i+1,0);
//				print_util_dbg_print("|olddistAC|");print_util_dbg_putfloat(old_distAC[i],5);
//				print_util_dbg_print("|distAC|");print_util_dbg_putfloat(distAC,5);
//				print_util_dbg_print("|angle_rep|");print_util_dbg_putfloat(angle_rep_radius,5);
//				print_util_dbg_print("|angle|");print_util_dbg_putfloat(waypoint_handler->fance_angle_list[i]*MATH_RAD_TO_DEG,5);
//				print_util_dbg_print("\n");
			}
		}
		old_distAC[i]=distAC;
		//fence repulsion

//		dist = detect_line(Alpoint,Blpoint,this->pos_est->last_gps_pos,V, gamma,I);
		dist[i] = detect_seg(A,B,C,S,V,I,J);

		if((dist[i] >= -(this->maxsens))&(dist[i] < this->maxsens))//		if(0)
		{
			float rep[3]={A[1]-B[1],B[0]-A[0],0.0};
			gftobftransform(C, S, rep);
			vectors_normalize(rep,rep);
			rep[1]=(rep[1]>=0?1:-1) ;

			float ratio = dist[i]/this->maxsens;

//			this->repulsion[0]+=0.0;
			rep[1]=(rep[1]>=0?1:-1); // 1 = clockwise / -1 = counterclockwise
			this->repulsion[1]+=-rep[1]*this->coef_roll*max_ang*interpolate(ratio,interp_type);
//			this->repulsion[2]+=0.0;
			detected++;
		}
		else
		{
//			pos_est->vel_bf[1]=0.0;
		}

	}
	//Cliping
	if(this->repulsion[1]>max_ang)
	{
		this->repulsion[1]=max_ang;
	}
	if(this->repulsion[1]<-max_ang)
	{
		this->repulsion[1]=-max_ang;
	}
	if(detected==0)
	{
		controls->rpy[ROLL] = 0.0;
	}


	return true;
}
float  Fence_CAS::interpolate(float r, int type) //type=x, 0: linear, 1: cos, 2:cos2
{
	if(type==0) // linear interpolation
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
	else if(type==1) // cos interpolation
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
	if(type==2) // cos2 interpolation
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
//	float temp0 = (S[0]-C[0])*rep[0] + (C[1]-S[1])*rep[1];
//	float temp1 = (S[1]-C[1])*rep[0] + (S[0]-C[0])*rep[1];
	float temp0 = (S[0]-C[0])*rep[0] + (S[1]-C[1])*rep[1];
	float temp1 = (S[1]-C[1])*rep[0] + (C[0]-S[0])*rep[1];
	rep[0]=temp0;
	rep[1]=temp1;
	rep[2]=0.0;
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
float Fence_CAS::get_repulsion(int axis)
{
	return this->repulsion[axis];
}
float Fence_CAS::get_max_angle(void)
{
	return 0.25*PI*(1-this->comfort);
}
