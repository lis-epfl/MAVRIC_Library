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


#include "simulation/wing_model.hpp"
#include <iostream>

extern "C"
{
#include "hal/common/time_keeper.hpp"
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Wing_model::Wing_model(float flap_angle,
	quat_t orientation,
	float x_position_bf,
	float y_position_bf,
	float z_position_bf,
	float area,
	float chord):
		flap_angle_(flap_angle),
		orientation_(orientation),
		area_(area),
		chord_(chord)
{
	position_bf_[0] = x_position_bf;
	position_bf_[1] = y_position_bf;
	position_bf_[2] = z_position_bf;
}

wing_model_forces_t Wing_model::compute_forces(quat_t wind_bf){
	quat_t wind_wf = quaternions_global_to_local(orientation_,wind_bf); //TODO : double check
	float aoa = atan2(-wind_wf.v[2], -wind_wf.v[0]); //Neglect the lateral wind
	float speed_sq = SQR(wind_wf.v[0]) + SQR(wind_wf.v[2]); //Neglect the lateral wind
	float speed = sqrt(speed_sq);
	float cl = get_cl(aoa,speed);
	float cd = get_cd(aoa,speed);
	float cm = get_cm(aoa,speed);
	printf("Cm: %f\n",cm);
	float density=1.225f; //Keep it constant for now
	float base = 0.5*density*speed_sq*area_;
	wing_model_forces_t forces_wf;
	forces_wf.torque[ROLL] = 0.0;
	forces_wf.torque[PITCH] = cm*chord_*base; //Positive when plane lift its nose
	forces_wf.torque[YAW] = 0.0;
	forces_wf.force[0] = -cd*base; //Drag is directed to the tail of the plane
	forces_wf.force[1] = 0.0;
	forces_wf.force[2] = -cl*base; //Lift is directed on the upward direction
	wing_model_forces_t forces_bf = forces_wing_to_bf(forces_wf);
	//printf("___________________\nAoa: %f, Cl: %f, Cd:%f, Cm:%f\nWF:\n Pitch Torque: %f, XForce:%f, ZForce:%f\n",aoa,cl,cd,cm,forces_wf.torque[PITCH],forces_wf.force[0],forces_wf.force[2]);
	//printf("BF:\n Roll Torque: %f, Pitch Torque: %f, Yaw Torque: %f\n XForce:%f, YForce:%f, ZForce:%f\n_____________________\n",forces_bf.torque[ROLL],forces_bf.torque[PITCH],forces_bf.torque[YAW],forces_bf.force[0],forces_bf.force[1],forces_bf.force[2]);
	return forces_bf;
}

void Wing_model::set_flap_angle(float angle){
	flap_angle_ = angle;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

float Wing_model::get_cl(float aoa, float speed)
{
	float s = sin(aoa);
	return 0.238746500064360 +
				2.409518274074305*s -
				1.524150734902402*s*s -
				3.265822324112502*s*s*s +
				3.139682594077404*s*s*s*s +
				1.091789171091257*s*s*s*s*s -
				1.946631374465694*s*s*s*s*s*s; //TODO: implement function
}

float Wing_model::get_cd(float aoa, float speed)
{
	float s = sin(aoa);
	return 0.022894020849831 -
	// return (0.152894020849831 -
				0.059723970421536*s +
				2.083877344767450*s*s +
				0.471968503907211*s*s*s -
				3.246828509758145*s*s*s*s -
		  	0.441748631551998*s*s*s*s*s +
				1.718122708953699*s*s*s*s*s*s; //TODO: implement function
}

float Wing_model::get_cm(float aoa, float speed)
{
	/*float s = sin(aoa);
	return -0.021578049878998 +
				0.047194268949306*s +
				0.011231572553684*s*s -
				1.099292373948442*s*s*s +
				0.141489785286803*s*s*s*s +
				0.819495548481242*s*s*s*s*s -
				0.135629725142560*s*s*s*s*s*s; //TODO: implement function*/
				return -aoa/3.141592f*180.0f/80.0f*0.25f+0.01;
}

wing_model_forces_t Wing_model::forces_wing_to_bf(wing_model_forces_t forces_wf)
{
	quat_t tmp;
	tmp.s = 0.0;
	tmp.v[0] = forces_wf.force[0];
	tmp.v[1] = forces_wf.force[1];
	tmp.v[2] = forces_wf.force[2];
	tmp = quaternions_local_to_global(orientation_,tmp);
	wing_model_forces_t forces_bf;
	forces_bf.force[0] = tmp.v[0];
	forces_bf.force[1] = tmp.v[1];
	forces_bf.force[2] = tmp.v[2];
	/*quat_t roll, pitch, yaw;
	roll.s = forces_wf.torque[ROLL]; roll.v[0]=1.0f; roll.v[1]=0.0f; roll.v[2]=0.0f; //TODO: double check how the rotation is done
	pitch.s = forces_wf.torque[PITCH]; pitch.v[0]=0.0f; pitch.v[1]=1.0f; pitch.v[2]=0.0f;
	yaw.s = forces_wf.torque[YAW]; yaw.v[0]=0.0f; yaw.v[1]=0.0f; yaw.v[2]=1.0f;
	roll = quaternions_local_to_global(orientation_,roll);
	pitch = quaternions_local_to_global(orientation_,pitch);
	yaw = quaternions_local_to_global(orientation_,yaw);
	forces_bf.torque[ROLL]  = roll.s*roll.v[0] + pitch.s*pitch.v[0] + yaw.s*yaw.v[0] + forces_bf.force[2]*position_bf_[1] - forces_bf.force[1]*position_bf_[2]; //Axis: front, right, down, with rotations: right hand
	forces_bf.torque[PITCH] = roll.s*roll.v[1] + pitch.s*pitch.v[1] + yaw.s*yaw.v[1] - forces_bf.force[2]*position_bf_[0] + forces_bf.force[0]*position_bf_[2];
	forces_bf.torque[YAW]   = roll.s*roll.v[2] + pitch.s*pitch.v[2] + yaw.s*yaw.v[2] - forces_bf.force[0]*position_bf_[1] + forces_bf.force[1]*position_bf_[0];*/
	quat_t torques_wf;
	torques_wf.s = 0.0f;
	for(int i=0; i<3; i++) torques_wf.v[i]=forces_wf.torque[i];
	quat_t torques_bf = quaternions_local_to_global(orientation_,torques_wf);
	for(int i=0; i<3; i++) forces_bf.torque[i]=torques_bf.v[i] + forces_bf.force[(i+2)%3]*position_bf_[(i+1)%3] - forces_bf.force[(i+1)%3]*position_bf_[(i+2)%3];
	return forces_bf;
}
