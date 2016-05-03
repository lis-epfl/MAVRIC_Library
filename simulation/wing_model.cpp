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
#include <fstream>

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
	float chord,
	int type):
		flap_angle_(flap_angle),
		orientation_(orientation),
		area_(area),
		chord_(chord),
		type_(type)
{
	position_bf_[0] = x_position_bf;
	position_bf_[1] = y_position_bf;
	position_bf_[2] = z_position_bf;
	init_lookup();
}

wing_model_forces_t Wing_model::compute_forces(float wind[3], float ang_rates[3]){
	//Compute the wind speed at the COG of the wing
	float wind_bf[3];
	wind_bf[0] = wind[0] - (ang_rates[1]*position_bf_[2]-ang_rates[2]*position_bf_[1]);
	wind_bf[1] = wind[1] - (ang_rates[2]*position_bf_[0]-ang_rates[0]*position_bf_[2]);
	wind_bf[2] = wind[2] - (ang_rates[0]*position_bf_[1]-ang_rates[1]*position_bf_[0]);

	//Global to local wind
	float wind_wf[3];
	quaternions_rotate_vector(quaternions_inverse(orientation_),wind_bf,wind_wf);
	float aoa = atan2(-wind_wf[2], -wind_wf[0]); //Neglect the lateral wind
	float speed_sq = SQR(wind_wf[0]) + SQR(wind_wf[2]); //Neglect the lateral wind
	float cl = get_cl(aoa);
	float cd = get_cd(aoa);
	float cm = get_cm(aoa);
	//printf("%f\n",aoa/PI*180);
	float density=1.225f; //Keep it constant for now
	float base = 0.5*density*speed_sq*area_;
	float lift = cl*base; //Doing this to improve the speed -> Correct??
	float drag = cd*base;
	float sinus = sin(aoa);
	float cosinus = cos(aoa);
	wing_model_forces_t forces_wf;
	forces_wf.torque[ROLL] = 0.0;
	forces_wf.torque[PITCH] = cm*chord_*base; //Positive when plane lift its nose
	forces_wf.torque[YAW] = 0.0;
	forces_wf.force[0] = -drag*cosinus+lift*sinus; //Drag and lift are // and orthogonal to the wind
	forces_wf.force[1] = 0.0;
	forces_wf.force[2] = -lift*cosinus-drag*sinus;
	wing_model_forces_t forces_bf = forces_wing_to_bf(forces_wf);
	return forces_bf;
}

void Wing_model::set_flap_angle(float angle){
	if(angle!=angle) //check for nan
	{
		flap_angle_ = 0.0f;
	}
	else if (angle>0.7f)//TODO fix better than that
	{
		flap_angle_=0.7f;
	}
	else if (angle<-0.7f)
	{
		flap_angle_=-0.7f;
	}
	else
	{
		flap_angle_ = angle;
	}
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

float Wing_model::get_cl(float aoa)
{
	float coeff=0.0f;
	if(type_ == 1) // Type 1 = zagi 12
	{
		printf("->%f\n", flap_angle_);
		aoa = (aoa+flap_angle_*0.15f)*57.2957f;//*180/pi -> Take flap angle into account and transform it in degrees
		if(aoa<-90)
		{
			coeff = lookup_Cl_[0];
		}
		else if(aoa>90)
		{
			coeff = lookup_Cl_[180];
		}
		else if(floor(aoa)==ceil(aoa))
		{
			coeff = lookup_Cl_[(int)floor(aoa)+90];
		}
		else
		{
			coeff = (aoa-floor(aoa))/(ceil(aoa)-floor(aoa))*(lookup_Cl_[(int)ceil(aoa)+90]-lookup_Cl_[(int)floor(aoa)+90])+lookup_Cl_[(int)floor(aoa)+90];
		}
		coeff += 0.4297f*flap_angle_;
	}
	else //Return the flat profile
	{
		coeff = 2.0f*PI*aoa;
	}
	return coeff;
}

float Wing_model::get_cd(float aoa)
{
	float coeff = 0.0f;
	if(type_ == 1) // Type 1 = zagi 12
	{
		aoa = (aoa+flap_angle_*0.15f)*57.2957f;//*180/pi -> Take flap angle into account and transform it in degrees
		if(aoa<-90)
		{
			coeff = lookup_Cd_[0];
		}
		else if(aoa>90)
		{
			coeff = lookup_Cd_[180];
		}
		else if(floor(aoa)==ceil(aoa))
		{
			coeff = lookup_Cd_[(int)floor(aoa)+90];
		}
		else
		{
			coeff = (aoa-floor(aoa))/(ceil(aoa)-floor(aoa))*(lookup_Cd_[(int)ceil(aoa)+90]-lookup_Cd_[(int)floor(aoa)+90])+lookup_Cd_[(int)floor(aoa)+90];
		}
		coeff += abs(0.0644577f*flap_angle_);
	}
	else //Return the flat profile
	{
		coeff = 1.28f*sin(aoa);
	}
	return coeff;
}

float Wing_model::get_cm(float aoa)
{
	float coeff = 0.0f;
	if(type_ == 1) // Type 1 = zagi 12
	{
		aoa = (aoa+flap_angle_*0.15f)*57.2957f;// *180/pi -> Take flap angle into account and transform it in degrees
		if(aoa<-90)
		{
			coeff = lookup_Cm_[0];
		}
		else if(aoa>90)
		{
			coeff = lookup_Cm_[180];
		}
		else if(floor(aoa)==ceil(aoa))
		{
			coeff = lookup_Cm_[(int)floor(aoa)+90];
		}
		else
		{
			coeff = (aoa-floor(aoa))/(ceil(aoa)-floor(aoa))*(lookup_Cm_[(int)ceil(aoa)+90]-lookup_Cm_[(int)floor(aoa)+90])+lookup_Cm_[(int)floor(aoa)+90];
		}
		coeff -= 0.214859f*flap_angle_;
	}
	else //Return the flat profile
	{
		coeff = 0.25f*(get_cl(aoa)*cos(aoa)+get_cd(aoa)*sin(aoa));
	}
	return coeff;
}

wing_model_forces_t Wing_model::forces_wing_to_bf(wing_model_forces_t forces_wf)
{
	wing_model_forces_t forces_bf;
	// Rotate the forces
	quaternions_rotate_vector(orientation_,forces_wf.force,forces_bf.force);
	// Rotate the torques
	quaternions_rotate_vector(orientation_,forces_wf.torque,forces_bf.torque);
	// Compute the torques due to the forces being applied away from COG
	for(int i=0; i<3; i++) forces_bf.torque[i]+= forces_bf.force[(i+2)%3]*position_bf_[(i+1)%3] - forces_bf.force[(i+1)%3]*position_bf_[(i+2)%3];
	return forces_bf;
}

void Wing_model::init_lookup()
{
	//Cl
	 lookup_Cl_[0] = -0.099154;
	 lookup_Cl_[1] = -0.112739;
	 lookup_Cl_[2] = -0.128425;
	 lookup_Cl_[3] = -0.141660;
	 lookup_Cl_[4] = -0.156783;
	 lookup_Cl_[5] = -0.170699;
	 lookup_Cl_[6] = -0.185203;
	 lookup_Cl_[7] = -0.200156;
	 lookup_Cl_[8] = -0.215458;
	 lookup_Cl_[9] = -0.230890;
	 lookup_Cl_[10] = -0.247198;
	 lookup_Cl_[11] = -0.261970;
	 lookup_Cl_[12] = -0.276913;
	 lookup_Cl_[13] = -0.292022;
	 lookup_Cl_[14] = -0.307318;
	 lookup_Cl_[15] = -0.322794;
	 lookup_Cl_[16] = -0.338439;
	 lookup_Cl_[17] = -0.354245;
	 lookup_Cl_[18] = -0.370201;
	 lookup_Cl_[19] = -0.386284;
	 lookup_Cl_[20] = -0.402478;
	 lookup_Cl_[21] = -0.418756;
	 lookup_Cl_[22] = -0.435085;
	 lookup_Cl_[23] = -0.451424;
	 lookup_Cl_[24] = -0.467734;
	 lookup_Cl_[25] = -0.483974;
	 lookup_Cl_[26] = -0.500104;
	 lookup_Cl_[27] = -0.516050;
	 lookup_Cl_[28] = -0.531722;
	 lookup_Cl_[29] = -0.547002;
	 lookup_Cl_[30] = -0.561777;
	 lookup_Cl_[31] = -0.576118;
	 lookup_Cl_[32] = -0.590171;
	 lookup_Cl_[33] = -0.604011;
	 lookup_Cl_[34] = -0.617653;
	 lookup_Cl_[35] = -0.631053;
	 lookup_Cl_[36] = -0.644171;
	 lookup_Cl_[37] = -0.656957;
	 lookup_Cl_[38] = -0.669354;
	 lookup_Cl_[39] = -0.681293;
	 lookup_Cl_[40] = -0.692693;
	 lookup_Cl_[41] = -0.703445;
	 lookup_Cl_[42] = -0.712642;
	 lookup_Cl_[43] = -0.721267;
	 lookup_Cl_[44] = -0.730237;
	 lookup_Cl_[45] = -0.738707;
	 lookup_Cl_[46] = -0.746747;
	 lookup_Cl_[47] = -0.754053;
	 lookup_Cl_[48] = -0.760553;
	 lookup_Cl_[49] = -0.766026;
	 lookup_Cl_[50] = -0.770648;
	 lookup_Cl_[51] = -0.774316;
	 lookup_Cl_[52] = -0.776593;
	 lookup_Cl_[53] = -0.777953;
	 lookup_Cl_[54] = -0.778078;
	 lookup_Cl_[55] = -0.776882;
	 lookup_Cl_[56] = -0.774255;
	 lookup_Cl_[57] = -0.770076;
	 lookup_Cl_[58] = -0.723268;
	 lookup_Cl_[59] = -0.699579;
	 lookup_Cl_[60] = -0.693466;
	 lookup_Cl_[61] = -0.683706;
	 lookup_Cl_[62] = -0.673224;
	 lookup_Cl_[63] = -0.660201;
	 lookup_Cl_[64] = -0.645757;
	 lookup_Cl_[65] = -0.628025;
	 lookup_Cl_[66] = -0.609150;
	 lookup_Cl_[67] = -0.588649;
	 lookup_Cl_[68] = -0.568871;
	 lookup_Cl_[69] = -0.546709;
	 lookup_Cl_[70] = -0.525062;
	 lookup_Cl_[71] = -0.504684;
	 lookup_Cl_[72] = -0.484900;
	 lookup_Cl_[73] = -0.463399;
	 lookup_Cl_[74] = -0.443568;
	 lookup_Cl_[75] = -0.427371;
	 lookup_Cl_[76] = -0.411703;
	 lookup_Cl_[77] = -0.398456;
	 lookup_Cl_[78] = -0.391637;
	 lookup_Cl_[79] = -0.390806;
	 lookup_Cl_[80] = -0.401964;
	 lookup_Cl_[81] = -0.434490;
	 lookup_Cl_[82] = -0.531087;
	 lookup_Cl_[83] = -0.613844;
	 lookup_Cl_[84] = -0.548873;
	 lookup_Cl_[85] = -0.424754;
	 lookup_Cl_[86] = -0.292417;
	 lookup_Cl_[87] = -0.118411;
	 lookup_Cl_[88] = 0.094584;
	 lookup_Cl_[89] = 0.248246;
	 lookup_Cl_[90] = 0.386868;
	 lookup_Cl_[91] = 0.527127;
	 lookup_Cl_[92] = 0.633913;
	 lookup_Cl_[93] = 0.712960;
	 lookup_Cl_[94] = 0.786209;
	 lookup_Cl_[95] = 0.858249;
	 lookup_Cl_[96] = 0.923183;
	 lookup_Cl_[97] = 0.972336;
	 lookup_Cl_[98] = 0.963124;
	 lookup_Cl_[99] = 0.909745;
	 lookup_Cl_[100] = 0.907774;
	 lookup_Cl_[101] = 0.924647;
	 lookup_Cl_[102] = 0.859901;
	 lookup_Cl_[103] = 0.771692;
	 lookup_Cl_[104] = 0.691953;
	 lookup_Cl_[105] = 0.651084;
	 lookup_Cl_[106] = 0.652737;
	 lookup_Cl_[107] = 0.660919;
	 lookup_Cl_[108] = 0.671773;
	 lookup_Cl_[109] = 0.686274;
	 lookup_Cl_[110] = 0.703388;
	 lookup_Cl_[111] = 0.718091;
	 lookup_Cl_[112] = 0.733618;
	 lookup_Cl_[113] = 0.746782;
	 lookup_Cl_[114] = 0.760850;
	 lookup_Cl_[115] = 0.774459;
	 lookup_Cl_[116] = 0.781616;
	 lookup_Cl_[117] = 0.784801;
	 lookup_Cl_[118] = 0.730325;
	 lookup_Cl_[119] = 0.760916;
	 lookup_Cl_[120] = 0.775647;
	 lookup_Cl_[121] = 0.786886;
	 lookup_Cl_[122] = 0.796269;
	 lookup_Cl_[123] = 0.802871;
	 lookup_Cl_[124] = 0.808284;
	 lookup_Cl_[125] = 0.811187;
	 lookup_Cl_[126] = 0.813136;
	 lookup_Cl_[127] = 0.813617;
	 lookup_Cl_[128] = 0.812626;
	 lookup_Cl_[129] = 0.810382;
	 lookup_Cl_[130] = 0.807090;
	 lookup_Cl_[131] = 0.802422;
	 lookup_Cl_[132] = 0.796558;
	 lookup_Cl_[133] = 0.789613;
	 lookup_Cl_[134] = 0.781644;
	 lookup_Cl_[135] = 0.772713;
	 lookup_Cl_[136] = 0.762955;
	 lookup_Cl_[137] = 0.752284;
	 lookup_Cl_[138] = 0.740985;
	 lookup_Cl_[139] = 0.728856;
	 lookup_Cl_[140] = 0.716136;
	 lookup_Cl_[141] = 0.703151;
	 lookup_Cl_[142] = 0.690536;
	 lookup_Cl_[143] = 0.676715;
	 lookup_Cl_[144] = 0.662541;
	 lookup_Cl_[145] = 0.647798;
	 lookup_Cl_[146] = 0.632584;
	 lookup_Cl_[147] = 0.616794;
	 lookup_Cl_[148] = 0.600317;
	 lookup_Cl_[149] = 0.583026;
	 lookup_Cl_[150] = 0.565056;
	 lookup_Cl_[151] = 0.546549;
	 lookup_Cl_[152] = 0.527684;
	 lookup_Cl_[153] = 0.508667;
	 lookup_Cl_[154] = 0.489328;
	 lookup_Cl_[155] = 0.469886;
	 lookup_Cl_[156] = 0.450350;
	 lookup_Cl_[157] = 0.430929;
	 lookup_Cl_[158] = 0.411591;
	 lookup_Cl_[159] = 0.392411;
	 lookup_Cl_[160] = 0.373436;
	 lookup_Cl_[161] = 0.354719;
	 lookup_Cl_[162] = 0.336358;
	 lookup_Cl_[163] = 0.318254;
	 lookup_Cl_[164] = 0.300364;
	 lookup_Cl_[165] = 0.282730;
	 lookup_Cl_[166] = 0.265363;
	 lookup_Cl_[167] = 0.248293;
	 lookup_Cl_[168] = 0.231525;
	 lookup_Cl_[169] = 0.215125;
	 lookup_Cl_[170] = 0.198873;
	 lookup_Cl_[171] = 0.183021;
	 lookup_Cl_[172] = 0.167150;
	 lookup_Cl_[173] = 0.151655;
	 lookup_Cl_[174] = 0.136103;
	 lookup_Cl_[175] = 0.121274;
	 lookup_Cl_[176] = 0.106857;
	 lookup_Cl_[177] = 0.091660;
	 lookup_Cl_[178] = 0.076896;
	 lookup_Cl_[179] = 0.061438;
	 lookup_Cl_[180] = 0.045905;

	//Cd
	 lookup_Cd_[0] = 0.616226;
	 lookup_Cd_[1] = 0.605101;
	 lookup_Cd_[2] = 0.592267;
	 lookup_Cd_[3] = 0.584714;
	 lookup_Cd_[4] = 0.575932;
	 lookup_Cd_[5] = 0.570808;
	 lookup_Cd_[6] = 0.565731;
	 lookup_Cd_[7] = 0.560812;
	 lookup_Cd_[8] = 0.555953;
	 lookup_Cd_[9] = 0.551342;
	 lookup_Cd_[10] = 0.546091;
	 lookup_Cd_[11] = 0.544047;
	 lookup_Cd_[12] = 0.542442;
	 lookup_Cd_[13] = 0.540703;
	 lookup_Cd_[14] = 0.538870;
	 lookup_Cd_[15] = 0.536925;
	 lookup_Cd_[16] = 0.534790;
	 lookup_Cd_[17] = 0.532367;
	 lookup_Cd_[18] = 0.529658;
	 lookup_Cd_[19] = 0.526603;
	 lookup_Cd_[20] = 0.523150;
	 lookup_Cd_[21] = 0.519253;
	 lookup_Cd_[22] = 0.514889;
	 lookup_Cd_[23] = 0.510070;
	 lookup_Cd_[24] = 0.504821;
	 lookup_Cd_[25] = 0.499193;
	 lookup_Cd_[26] = 0.493244;
	 lookup_Cd_[27] = 0.486986;
	 lookup_Cd_[28] = 0.480496;
	 lookup_Cd_[29] = 0.473932;
	 lookup_Cd_[30] = 0.467474;
	 lookup_Cd_[31] = 0.461183;
	 lookup_Cd_[32] = 0.454958;
	 lookup_Cd_[33] = 0.448762;
	 lookup_Cd_[34] = 0.442606;
	 lookup_Cd_[35] = 0.436552;
	 lookup_Cd_[36] = 0.430631;
	 lookup_Cd_[37] = 0.424881;
	 lookup_Cd_[38] = 0.419330;
	 lookup_Cd_[39] = 0.413993;
	 lookup_Cd_[40] = 0.408858;
	 lookup_Cd_[41] = 0.403832;
	 lookup_Cd_[42] = 0.398094;
	 lookup_Cd_[43] = 0.392329;
	 lookup_Cd_[44] = 0.388710;
	 lookup_Cd_[45] = 0.385763;
	 lookup_Cd_[46] = 0.383009;
	 lookup_Cd_[47] = 0.380421;
	 lookup_Cd_[48] = 0.377941;
	 lookup_Cd_[49] = 0.375694;
	 lookup_Cd_[50] = 0.373221;
	 lookup_Cd_[51] = 0.370874;
	 lookup_Cd_[52] = 0.368345;
	 lookup_Cd_[53] = 0.365885;
	 lookup_Cd_[54] = 0.363394;
	 lookup_Cd_[55] = 0.360867;
	 lookup_Cd_[56] = 0.358269;
	 lookup_Cd_[57] = 0.355609;
	 lookup_Cd_[58] = 0.377413;
	 lookup_Cd_[59] = 0.380071;
	 lookup_Cd_[60] = 0.369342;
	 lookup_Cd_[61] = 0.356129;
	 lookup_Cd_[62] = 0.341321;
	 lookup_Cd_[63] = 0.326662;
	 lookup_Cd_[64] = 0.313257;
	 lookup_Cd_[65] = 0.297689;
	 lookup_Cd_[66] = 0.282891;
	 lookup_Cd_[67] = 0.268838;
	 lookup_Cd_[68] = 0.255882;
	 lookup_Cd_[69] = 0.242366;
	 lookup_Cd_[70] = 0.229473;
	 lookup_Cd_[71] = 0.215328;
	 lookup_Cd_[72] = 0.204806;
	 lookup_Cd_[73] = 0.191653;
	 lookup_Cd_[74] = 0.180347;
	 lookup_Cd_[75] = 0.169067;
	 lookup_Cd_[76] = 0.154075;
	 lookup_Cd_[77] = 0.140030;
	 lookup_Cd_[78] = 0.126520;
	 lookup_Cd_[79] = 0.111018;
	 lookup_Cd_[80] = 0.095671;
	 lookup_Cd_[81] = 0.076851;
	 lookup_Cd_[82] = 0.063314;
	 lookup_Cd_[83] = 0.041805;
	 lookup_Cd_[84] = 0.027318;
	 lookup_Cd_[85] = 0.022834;
	 lookup_Cd_[86] = 0.018285;
	 lookup_Cd_[87] = 0.015051;
	 lookup_Cd_[88] = 0.014584;
	 lookup_Cd_[89] = 0.014525;
	 lookup_Cd_[90] = 0.014731;
	 lookup_Cd_[91] = 0.015317;
	 lookup_Cd_[92] = 0.015581;
	 lookup_Cd_[93] = 0.016644;
	 lookup_Cd_[94] = 0.017845;
	 lookup_Cd_[95] = 0.019090;
	 lookup_Cd_[96] = 0.019226;
	 lookup_Cd_[97] = 0.019522;
	 lookup_Cd_[98] = 0.022829;
	 lookup_Cd_[99] = 0.028021;
	 lookup_Cd_[100] = 0.037542;
	 lookup_Cd_[101] = 0.053362;
	 lookup_Cd_[102] = 0.069402;
	 lookup_Cd_[103] = 0.091426;
	 lookup_Cd_[104] = 0.122927;
	 lookup_Cd_[105] = 0.156062;
	 lookup_Cd_[106] = 0.176655;
	 lookup_Cd_[107] = 0.192476;
	 lookup_Cd_[108] = 0.206840;
	 lookup_Cd_[109] = 0.220895;
	 lookup_Cd_[110] = 0.238920;
	 lookup_Cd_[111] = 0.257824;
	 lookup_Cd_[112] = 0.273190;
	 lookup_Cd_[113] = 0.289977;
	 lookup_Cd_[114] = 0.306812;
	 lookup_Cd_[115] = 0.324471;
	 lookup_Cd_[116] = 0.342445;
	 lookup_Cd_[117] = 0.359572;
	 lookup_Cd_[118] = 0.359779;
	 lookup_Cd_[119] = 0.363621;
	 lookup_Cd_[120] = 0.372740;
	 lookup_Cd_[121] = 0.382868;
	 lookup_Cd_[122] = 0.393001;
	 lookup_Cd_[123] = 0.402630;
	 lookup_Cd_[124] = 0.412285;
	 lookup_Cd_[125] = 0.421450;
	 lookup_Cd_[126] = 0.430638;
	 lookup_Cd_[127] = 0.439585;
	 lookup_Cd_[128] = 0.448243;
	 lookup_Cd_[129] = 0.456595;
	 lookup_Cd_[130] = 0.464679;
	 lookup_Cd_[131] = 0.472341;
	 lookup_Cd_[132] = 0.479575;
	 lookup_Cd_[133] = 0.486378;
	 lookup_Cd_[134] = 0.492729;
	 lookup_Cd_[135] = 0.498599;
	 lookup_Cd_[136] = 0.504008;
	 lookup_Cd_[137] = 0.508905;
	 lookup_Cd_[138] = 0.513362;
	 lookup_Cd_[139] = 0.517326;
	 lookup_Cd_[140] = 0.520918;
	 lookup_Cd_[141] = 0.524817;
	 lookup_Cd_[142] = 0.530815;
	 lookup_Cd_[143] = 0.535368;
	 lookup_Cd_[144] = 0.539306;
	 lookup_Cd_[145] = 0.542552;
	 lookup_Cd_[146] = 0.545199;
	 lookup_Cd_[147] = 0.547193;
	 lookup_Cd_[148] = 0.548310;
	 lookup_Cd_[149] = 0.548341;
	 lookup_Cd_[150] = 0.547438;
	 lookup_Cd_[151] = 0.545799;
	 lookup_Cd_[152] = 0.543647;
	 lookup_Cd_[153] = 0.541169;
	 lookup_Cd_[154] = 0.538362;
	 lookup_Cd_[155] = 0.535416;
	 lookup_Cd_[156] = 0.532429;
	 lookup_Cd_[157] = 0.529516;
	 lookup_Cd_[158] = 0.526773;
	 lookup_Cd_[159] = 0.524183;
	 lookup_Cd_[160] = 0.521893;
	 lookup_Cd_[161] = 0.520126;
	 lookup_Cd_[162] = 0.519054;
	 lookup_Cd_[163] = 0.518410;
	 lookup_Cd_[164] = 0.517962;
	 lookup_Cd_[165] = 0.517733;
	 lookup_Cd_[166] = 0.517577;
	 lookup_Cd_[167] = 0.517873;
	 lookup_Cd_[168] = 0.518573;
	 lookup_Cd_[169] = 0.521406;
	 lookup_Cd_[170] = 0.522215;
	 lookup_Cd_[171] = 0.523685;
	 lookup_Cd_[172] = 0.527204;
	 lookup_Cd_[173] = 0.531156;
	 lookup_Cd_[174] = 0.537032;
	 lookup_Cd_[175] = 0.542750;
	 lookup_Cd_[176] = 0.550748;
	 lookup_Cd_[177] = 0.559660;
	 lookup_Cd_[178] = 0.570250;
	 lookup_Cd_[179] = 0.581817;
	 lookup_Cd_[180] = 0.593485;

	//Cm
	 lookup_Cm_[0] = 0.325772;
	 lookup_Cm_[1] = 0.317688;
	 lookup_Cm_[2] = 0.308874;
	 lookup_Cm_[3] = 0.300626;
	 lookup_Cm_[4] = 0.292501;
	 lookup_Cm_[5] = 0.285086;
	 lookup_Cm_[6] = 0.278353;
	 lookup_Cm_[7] = 0.272568;
	 lookup_Cm_[8] = 0.267690;
	 lookup_Cm_[9] = 0.263541;
	 lookup_Cm_[10] = 0.259806;
	 lookup_Cm_[11] = 0.256545;
	 lookup_Cm_[12] = 0.253719;
	 lookup_Cm_[13] = 0.251464;
	 lookup_Cm_[14] = 0.249792;
	 lookup_Cm_[15] = 0.248677;
	 lookup_Cm_[16] = 0.248087;
	 lookup_Cm_[17] = 0.247989;
	 lookup_Cm_[18] = 0.248345;
	 lookup_Cm_[19] = 0.249089;
	 lookup_Cm_[20] = 0.250173;
	 lookup_Cm_[21] = 0.251536;
	 lookup_Cm_[22] = 0.253106;
	 lookup_Cm_[23] = 0.254804;
	 lookup_Cm_[24] = 0.256542;
	 lookup_Cm_[25] = 0.258229;
	 lookup_Cm_[26] = 0.259764;
	 lookup_Cm_[27] = 0.261036;
	 lookup_Cm_[28] = 0.261907;
	 lookup_Cm_[29] = 0.262197;
	 lookup_Cm_[30] = 0.261755;
	 lookup_Cm_[31] = 0.260755;
	 lookup_Cm_[32] = 0.259515;
	 lookup_Cm_[33] = 0.258195;
	 lookup_Cm_[34] = 0.256845;
	 lookup_Cm_[35] = 0.255408;
	 lookup_Cm_[36] = 0.253865;
	 lookup_Cm_[37] = 0.252185;
	 lookup_Cm_[38] = 0.250337;
	 lookup_Cm_[39] = 0.248294;
	 lookup_Cm_[40] = 0.246032;
	 lookup_Cm_[41] = 0.243558;
	 lookup_Cm_[42] = 0.240829;
	 lookup_Cm_[43] = 0.237815;
	 lookup_Cm_[44] = 0.234318;
	 lookup_Cm_[45] = 0.230431;
	 lookup_Cm_[46] = 0.226665;
	 lookup_Cm_[47] = 0.222714;
	 lookup_Cm_[48] = 0.218579;
	 lookup_Cm_[49] = 0.213946;
	 lookup_Cm_[50] = 0.209437;
	 lookup_Cm_[51] = 0.204658;
	 lookup_Cm_[52] = 0.199507;
	 lookup_Cm_[53] = 0.194308;
	 lookup_Cm_[54] = 0.188831;
	 lookup_Cm_[55] = 0.183034;
	 lookup_Cm_[56] = 0.176873;
	 lookup_Cm_[57] = 0.170268;
	 lookup_Cm_[58] = 0.101594;
	 lookup_Cm_[59] = 0.064811;
	 lookup_Cm_[60] = 0.059214;
	 lookup_Cm_[61] = 0.050822;
	 lookup_Cm_[62] = 0.046928;
	 lookup_Cm_[63] = 0.042645;
	 lookup_Cm_[64] = 0.038025;
	 lookup_Cm_[65] = 0.033527;
	 lookup_Cm_[66] = 0.028857;
	 lookup_Cm_[67] = 0.023450;
	 lookup_Cm_[68] = 0.018543;
	 lookup_Cm_[69] = 0.013760;
	 lookup_Cm_[70] = 0.009035;
	 lookup_Cm_[71] = 0.005361;
	 lookup_Cm_[72] = 0.000244;
	 lookup_Cm_[73] = -0.003642;
	 lookup_Cm_[74] = -0.007996;
	 lookup_Cm_[75] = -0.012469;
	 lookup_Cm_[76] = -0.015986;
	 lookup_Cm_[77] = -0.019498;
	 lookup_Cm_[78] = -0.023825;
	 lookup_Cm_[79] = -0.028678;
	 lookup_Cm_[80] = -0.033663;
	 lookup_Cm_[81] = -0.039692;
	 lookup_Cm_[82] = -0.031715;
	 lookup_Cm_[83] = -0.009591;
	 lookup_Cm_[84] = -0.004312;
	 lookup_Cm_[85] = -0.009513;
	 lookup_Cm_[86] = -0.014916;
	 lookup_Cm_[87] = -0.024677;
	 lookup_Cm_[88] = -0.045148;
	 lookup_Cm_[89] = -0.054974;
	 lookup_Cm_[90] = -0.062215;
	 lookup_Cm_[91] = -0.069577;
	 lookup_Cm_[92] = -0.070878;
	 lookup_Cm_[93] = -0.067498;
	 lookup_Cm_[94] = -0.062751;
	 lookup_Cm_[95] = -0.056828;
	 lookup_Cm_[96] = -0.048942;
	 lookup_Cm_[97] = -0.036513;
	 lookup_Cm_[98] = -0.015264;
	 lookup_Cm_[99] = 0.010007;
	 lookup_Cm_[100] = 0.023633;
	 lookup_Cm_[101] = 0.034488;
	 lookup_Cm_[102] = 0.039249;
	 lookup_Cm_[103] = 0.031654;
	 lookup_Cm_[104] = 0.016126;
	 lookup_Cm_[105] = 0.001426;
	 lookup_Cm_[106] = -0.004603;
	 lookup_Cm_[107] = -0.010411;
	 lookup_Cm_[108] = -0.016184;
	 lookup_Cm_[109] = -0.021182;
	 lookup_Cm_[110] = -0.025979;
	 lookup_Cm_[111] = -0.031216;
	 lookup_Cm_[112] = -0.036476;
	 lookup_Cm_[113] = -0.041329;
	 lookup_Cm_[114] = -0.046611;
	 lookup_Cm_[115] = -0.051710;
	 lookup_Cm_[116] = -0.056494;
	 lookup_Cm_[117] = -0.060990;
	 lookup_Cm_[118] = -0.056848;
	 lookup_Cm_[119] = -0.118220;
	 lookup_Cm_[120] = -0.131838;
	 lookup_Cm_[121] = -0.142269;
	 lookup_Cm_[122] = -0.151631;
	 lookup_Cm_[123] = -0.159705;
	 lookup_Cm_[124] = -0.167311;
	 lookup_Cm_[125] = -0.174078;
	 lookup_Cm_[126] = -0.180625;
	 lookup_Cm_[127] = -0.186759;
	 lookup_Cm_[128] = -0.192487;
	 lookup_Cm_[129] = -0.197909;
	 lookup_Cm_[130] = -0.203111;
	 lookup_Cm_[131] = -0.207941;
	 lookup_Cm_[132] = -0.212478;
	 lookup_Cm_[133] = -0.216751;
	 lookup_Cm_[134] = -0.220772;
	 lookup_Cm_[135] = -0.224556;
	 lookup_Cm_[136] = -0.228141;
	 lookup_Cm_[137] = -0.231478;
	 lookup_Cm_[138] = -0.234662;
	 lookup_Cm_[139] = -0.237604;
	 lookup_Cm_[140] = -0.240345;
	 lookup_Cm_[141] = -0.242829;
	 lookup_Cm_[142] = -0.244834;
	 lookup_Cm_[143] = -0.246731;
	 lookup_Cm_[144] = -0.248613;
	 lookup_Cm_[145] = -0.250396;
	 lookup_Cm_[146] = -0.252063;
	 lookup_Cm_[147] = -0.253495;
	 lookup_Cm_[148] = -0.254473;
	 lookup_Cm_[149] = -0.254768;
	 lookup_Cm_[150] = -0.254453;
	 lookup_Cm_[151] = -0.253632;
	 lookup_Cm_[152] = -0.252450;
	 lookup_Cm_[153] = -0.251082;
	 lookup_Cm_[154] = -0.249362;
	 lookup_Cm_[155] = -0.247483;
	 lookup_Cm_[156] = -0.245485;
	 lookup_Cm_[157] = -0.243490;
	 lookup_Cm_[158] = -0.241527;
	 lookup_Cm_[159] = -0.239650;
	 lookup_Cm_[160] = -0.237898;
	 lookup_Cm_[161] = -0.236392;
	 lookup_Cm_[162] = -0.235070;
	 lookup_Cm_[163] = -0.234036;
	 lookup_Cm_[164] = -0.233410;
	 lookup_Cm_[165] = -0.233331;
	 lookup_Cm_[166] = -0.233747;
	 lookup_Cm_[167] = -0.234820;
	 lookup_Cm_[168] = -0.236480;
	 lookup_Cm_[169] = -0.238475;
	 lookup_Cm_[170] = -0.241542;
	 lookup_Cm_[171] = -0.245387;
	 lookup_Cm_[172] = -0.250236;
	 lookup_Cm_[173] = -0.256111;
	 lookup_Cm_[174] = -0.263259;
	 lookup_Cm_[175] = -0.271717;
	 lookup_Cm_[176] = -0.280970;
	 lookup_Cm_[177] = -0.291003;
	 lookup_Cm_[178] = -0.300661;
	 lookup_Cm_[179] = -0.308165;
	 lookup_Cm_[180] = -0.312962;
}
