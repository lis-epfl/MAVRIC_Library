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
	forces_wf.force[X] = -drag*cosinus+lift*sinus; //Drag and lift are // and orthogonal to the wind
	forces_wf.force[Y] = 0.0;
	forces_wf.force[Z] = -lift*cosinus-drag*sinus;
	wing_model_forces_t forces_bf = forces_wing_to_bf(forces_wf);
	return forces_bf;
}

void Wing_model::set_flap_angle(float angle){
	flap_angle_ = angle;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

float Wing_model::get_cl(float aoa)
{
	float coeff=0.0f;
	float angle=0.0f;
	float angle2=0.0f;
	float angle3=0.0f;
	float angle4=0.0f;
	if(type_ == 1) // Type 1 = zagi 12
	{
		aoa = (aoa+flap_angle_*0.15f)*57.2957f;//57.29 = 180/pi -> Take flap angle into account and transform it in degrees
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
		//coeff = 2.0f*PI*aoa;
		angle = fabs(aoa*57.2957f);
		angle2 = angle*angle;
		angle3 = angle2*angle;
		angle4 = angle3*angle;
		coeff = -0.000000092f*angle4+0.000019403f*angle3-0.001668105f*angle2+0.060127605f*angle;
		if(aoa<0)
		{
			coeff = -coeff;
		}

	}
	return coeff;
}

float Wing_model::get_cd(float aoa)
{
	float coeff = 0.0f;
	float angle=0.0f;
	float angle2=0.0f;
	float angle3=0.0f;
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
		coeff += fabs(0.0644577f*flap_angle_);
	}
	else //Return the flat profile
	{
		//coeff = 1.28f*sin(fabs(aoa));
		angle = fabs(aoa*57.2957f);
		angle2 = angle*angle;
		angle3 = angle2*angle;
		coeff = -0.000002950f*angle3+0.000474150f*angle2-0.002426600f*angle+0.153067920f;
	}
	return coeff;
}

float Wing_model::get_cm(float aoa)
{
	float angle=0.0f;
	float angle2=0.0f;
	float angle3=0.0f;
	float h=0.0f;
	float coeff = 0.0f;
	float coeff1 = 0.0f;
	float coeff2 = 0.0f;
	int LUTlow = 0;
	int LUThigh = 1;
	float lowBound = -0.69813170079f; //-40 degrees
	float range = 0.52359877559f; // 30 degrees
	if(type_ == 1) // Type 1 = zagi 12
	{
		//Determine which LUTs to use
		if(flap_angle_ < -0.17453292519f) //-40 to -10 degrees
		{
			LUTlow = 0;
			LUThigh = 1;
			lowBound = -0.69813170079f; //-40 degrees
			range = 0.52359877559f; // 30 degrees
		}
		else if(flap_angle_ < 0.17453292519f) //-10 to 10 degrees
		{
			LUTlow = 1;
			LUThigh = 2;
			lowBound = -0.17453292519f; //-10 degrees
			range = 0.34906585039f; // 20 degrees
		}
		else //10 to 40 degrees
		{
			LUTlow = 2;
			LUThigh = 3;
			lowBound = 0.17453292519f; //10 degrees
			range = 0.52359877559f; // 30 degrees
		}

		aoa *= 57.2957f; //Transform it in degrees
		//Comute the interpolation for the aoa
		if(aoa<-90)
		{
			coeff1 = lookup_Cm_[LUTlow][0];
			coeff2 = lookup_Cm_[LUThigh][0];
		}
		else if(aoa>90)
		{
			coeff1 = lookup_Cm_[LUTlow][180];
			coeff2 = lookup_Cm_[LUThigh][180];
		}
		else if(floor(aoa)==ceil(aoa))
		{
			coeff1 = lookup_Cm_[LUTlow][(int)floor(aoa)+90];
			coeff2 = lookup_Cm_[LUThigh][(int)floor(aoa)+90];
		}
		else
		{
			coeff1 = (aoa-floor(aoa))/(ceil(aoa)-floor(aoa))*(lookup_Cm_[LUTlow][(int)ceil(aoa)+90]-lookup_Cm_[LUTlow][(int)floor(aoa)+90])+lookup_Cm_[LUTlow][(int)floor(aoa)+90];
			coeff2 = (aoa-floor(aoa))/(ceil(aoa)-floor(aoa))*(lookup_Cm_[LUThigh][(int)ceil(aoa)+90]-lookup_Cm_[LUThigh][(int)floor(aoa)+90])+lookup_Cm_[LUThigh][(int)floor(aoa)+90];
		}

		//Compute the interpolation for the flap angles
		coeff = (coeff2-coeff1)*(flap_angle_-lowBound)/range + coeff1;
	}
	else //Return the flat profile
	{
		angle = fabs(aoa*57.2957f);
		angle2 = angle*angle;
		angle3 = angle2*angle;
		h = 0.000000765f*angle3-0.000133996f*angle2+0.009433786f*angle+0.181632486f;
		coeff = (get_cl(aoa)*cos(aoa)+get_cd(aoa)*sin(aoa))*(0.146f*h-0.073f);
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
	for(int i=0; i<3; i++) forces_bf.torque[i] += forces_bf.force[(i+2)%3]*position_bf_[(i+1)%3] - forces_bf.force[(i+1)%3]*position_bf_[(i+2)%3];
	return forces_bf;
}

void Wing_model::init_lookup() //Initialization of the lookup tables
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
	 lookup_Cm_[0][0] = 0.512059;
	 lookup_Cm_[0][1] = 0.507140;
	 lookup_Cm_[0][2] = 0.502221;
	 lookup_Cm_[0][3] = 0.497301;
	 lookup_Cm_[0][4] = 0.492382;
	 lookup_Cm_[0][5] = 0.487463;
	 lookup_Cm_[0][6] = 0.482544;
	 lookup_Cm_[0][7] = 0.477624;
	 lookup_Cm_[0][8] = 0.472705;
	 lookup_Cm_[0][9] = 0.467786;
	 lookup_Cm_[0][10] = 0.462867;
	 lookup_Cm_[0][11] = 0.457947;
	 lookup_Cm_[0][12] = 0.453028;
	 lookup_Cm_[0][13] = 0.448109;
	 lookup_Cm_[0][14] = 0.443190;
	 lookup_Cm_[0][15] = 0.438270;
	 lookup_Cm_[0][16] = 0.433351;
	 lookup_Cm_[0][17] = 0.428432;
	 lookup_Cm_[0][18] = 0.423513;
	 lookup_Cm_[0][19] = 0.418593;
	 lookup_Cm_[0][20] = 0.413674;
	 lookup_Cm_[0][21] = 0.408755;
	 lookup_Cm_[0][22] = 0.403836;
	 lookup_Cm_[0][23] = 0.398916;
	 lookup_Cm_[0][24] = 0.393997;
	 lookup_Cm_[0][25] = 0.389078;
	 lookup_Cm_[0][26] = 0.384159;
	 lookup_Cm_[0][27] = 0.379239;
	 lookup_Cm_[0][28] = 0.374320;
	 lookup_Cm_[0][29] = 0.369401;
	 lookup_Cm_[0][30] = 0.364482;
	 lookup_Cm_[0][31] = 0.359562;
	 lookup_Cm_[0][32] = 0.354643;
	 lookup_Cm_[0][33] = 0.349724;
	 lookup_Cm_[0][34] = 0.344805;
	 lookup_Cm_[0][35] = 0.339885;
	 lookup_Cm_[0][36] = 0.334966;
	 lookup_Cm_[0][37] = 0.330047;
	 lookup_Cm_[0][38] = 0.325128;
	 lookup_Cm_[0][39] = 0.320208;
	 lookup_Cm_[0][40] = 0.315289;
	 lookup_Cm_[0][41] = 0.310370;
	 lookup_Cm_[0][42] = 0.305451;
	 lookup_Cm_[0][43] = 0.300531;
	 lookup_Cm_[0][44] = 0.295612;
	 lookup_Cm_[0][45] = 0.290693;
	 lookup_Cm_[0][46] = 0.285774;
	 lookup_Cm_[0][47] = 0.280854;
	 lookup_Cm_[0][48] = 0.275935;
	 lookup_Cm_[0][49] = 0.271016;
	 lookup_Cm_[0][50] = 0.266097;
	 lookup_Cm_[0][51] = 0.261177;
	 lookup_Cm_[0][52] = 0.256258;
	 lookup_Cm_[0][53] = 0.251339;
	 lookup_Cm_[0][54] = 0.246420;
	 lookup_Cm_[0][55] = 0.241500;
	 lookup_Cm_[0][56] = 0.236581;
	 lookup_Cm_[0][57] = 0.231662;
	 lookup_Cm_[0][58] = 0.226743;
	 lookup_Cm_[0][59] = 0.221823;
	 lookup_Cm_[0][60] = 0.216904;
	 lookup_Cm_[0][61] = 0.211985;
	 lookup_Cm_[0][62] = 0.207066;
	 lookup_Cm_[0][63] = 0.202146;
	 lookup_Cm_[0][64] = 0.197227;
	 lookup_Cm_[0][65] = 0.192308;
	 lookup_Cm_[0][66] = 0.187389;
	 lookup_Cm_[0][67] = 0.182469;
	 lookup_Cm_[0][68] = 0.177550;
	 lookup_Cm_[0][69] = 0.172631;
	 lookup_Cm_[0][70] = 0.167712;
	 lookup_Cm_[0][71] = 0.162101;
	 lookup_Cm_[0][72] = 0.156605;
	 lookup_Cm_[0][73] = 0.150578;
	 lookup_Cm_[0][74] = 0.145764;
	 lookup_Cm_[0][75] = 0.140753;
	 lookup_Cm_[0][76] = 0.134230;
	 lookup_Cm_[0][77] = 0.131413;
	 lookup_Cm_[0][78] = 0.127446;
	 lookup_Cm_[0][79] = 0.119850;
	 lookup_Cm_[0][80] = 0.116225;
	 lookup_Cm_[0][81] = 0.111997;
	 lookup_Cm_[0][82] = 0.107187;
	 lookup_Cm_[0][83] = 0.098170;
	 lookup_Cm_[0][84] = 0.089683;
	 lookup_Cm_[0][85] = 0.070996;
	 lookup_Cm_[0][86] = 0.064063;
	 lookup_Cm_[0][87] = 0.072534;
	 lookup_Cm_[0][88] = 0.091043;
	 lookup_Cm_[0][89] = 0.101056;
	 lookup_Cm_[0][90] = 0.085980;
	 lookup_Cm_[0][91] = 0.083927;
	 lookup_Cm_[0][92] = 0.082027;
	 lookup_Cm_[0][93] = 0.064796;
	 lookup_Cm_[0][94] = 0.070265;
	 lookup_Cm_[0][95] = 0.077997;
	 lookup_Cm_[0][96] = 0.088691;
	 lookup_Cm_[0][97] = 0.076557;
	 lookup_Cm_[0][98] = 0.075813;
	 lookup_Cm_[0][99] = 0.076616;
	 lookup_Cm_[0][100] = 0.080428;
	 lookup_Cm_[0][101] = 0.089882;
	 lookup_Cm_[0][102] = 0.101873;
	 lookup_Cm_[0][103] = 0.075905;
	 lookup_Cm_[0][104] = 0.048381;
	 lookup_Cm_[0][105] = 0.042933;
	 lookup_Cm_[0][106] = 0.036180;
	 lookup_Cm_[0][107] = 0.029623;
	 lookup_Cm_[0][108] = 0.029895;
	 lookup_Cm_[0][109] = 0.029170;
	 lookup_Cm_[0][110] = 0.028392;
	 lookup_Cm_[0][111] = 0.026508;
	 lookup_Cm_[0][112] = 0.022449;
	 lookup_Cm_[0][113] = 0.024755;
	 lookup_Cm_[0][114] = 0.022622;
	 lookup_Cm_[0][115] = 0.022188;
	 lookup_Cm_[0][116] = 0.019386;
	 lookup_Cm_[0][117] = 0.017980;
	 lookup_Cm_[0][118] = 0.018056;
	 lookup_Cm_[0][119] = 0.016630;
	 lookup_Cm_[0][120] = 0.014760;
	 lookup_Cm_[0][121] = 0.011194;
	 lookup_Cm_[0][122] = 0.009073;
	 lookup_Cm_[0][123] = 0.006586;
	 lookup_Cm_[0][124] = 0.004258;
	 lookup_Cm_[0][125] = 0.002404;
	 lookup_Cm_[0][126] = 0.002769;
	 lookup_Cm_[0][127] = 0.001568;
	 lookup_Cm_[0][128] = 0.000008;
	 lookup_Cm_[0][129] = 0.003405;
	 lookup_Cm_[0][130] = 0.005821;
	 lookup_Cm_[0][131] = -0.000293;
	 lookup_Cm_[0][132] = -0.005127;
	 lookup_Cm_[0][133] = -0.009662;
	 lookup_Cm_[0][134] = -0.011667;
	 lookup_Cm_[0][135] = -0.013825;
	 lookup_Cm_[0][136] = -0.015730;
	 lookup_Cm_[0][137] = -0.017398;
	 lookup_Cm_[0][138] = -0.020185;
	 lookup_Cm_[0][139] = -0.022198;
	 lookup_Cm_[0][140] = -0.023659;
	 lookup_Cm_[0][141] = -0.025595;
	 lookup_Cm_[0][142] = -0.023410;
	 lookup_Cm_[0][143] = -0.024720;
	 lookup_Cm_[0][144] = -0.026299;
	 lookup_Cm_[0][145] = -0.027909;
	 lookup_Cm_[0][146] = -0.029002;
	 lookup_Cm_[0][147] = -0.031443;
	 lookup_Cm_[0][148] = -0.032585;
	 lookup_Cm_[0][149] = -0.037194;
	 lookup_Cm_[0][150] = -0.041088;
	 lookup_Cm_[0][151] = -0.042633;
	 lookup_Cm_[0][152] = -0.043486;
	 lookup_Cm_[0][153] = -0.049023;
	 lookup_Cm_[0][154] = -0.053885;
	 lookup_Cm_[0][155] = -0.046669;
	 lookup_Cm_[0][156] = -0.064869;
	 lookup_Cm_[0][157] = 0.005943;
	 lookup_Cm_[0][158] = 0.002510;
	 lookup_Cm_[0][159] = -0.003565;
	 lookup_Cm_[0][160] = -0.010812;
	 lookup_Cm_[0][161] = -0.019888;
	 lookup_Cm_[0][162] = -0.031212;
	 lookup_Cm_[0][163] = -0.042319;
	 lookup_Cm_[0][164] = -0.053658;
	 lookup_Cm_[0][165] = -0.065028;
	 lookup_Cm_[0][166] = -0.076417;
	 lookup_Cm_[0][167] = -0.087447;
	 lookup_Cm_[0][168] = -0.097898;
	 lookup_Cm_[0][169] = -0.108349;
	 lookup_Cm_[0][170] = -0.118800;
	 lookup_Cm_[0][171] = -0.129251;
	 lookup_Cm_[0][172] = -0.139702;
	 lookup_Cm_[0][173] = -0.150153;
	 lookup_Cm_[0][174] = -0.160604;
	 lookup_Cm_[0][175] = -0.171055;
	 lookup_Cm_[0][176] = -0.181506;
	 lookup_Cm_[0][177] = -0.191957;
	 lookup_Cm_[0][178] = -0.202408;
	 lookup_Cm_[0][179] = -0.212859;
	 lookup_Cm_[0][180] = -0.223309;
	 lookup_Cm_[1][0] = 0.353180;
	 lookup_Cm_[1][1] = 0.348351;
	 lookup_Cm_[1][2] = 0.344534;
	 lookup_Cm_[1][3] = 0.340387;
	 lookup_Cm_[1][4] = 0.336246;
	 lookup_Cm_[1][5] = 0.330748;
	 lookup_Cm_[1][6] = 0.324082;
	 lookup_Cm_[1][7] = 0.317014;
	 lookup_Cm_[1][8] = 0.309451;
	 lookup_Cm_[1][9] = 0.301722;
	 lookup_Cm_[1][10] = 0.294918;
	 lookup_Cm_[1][11] = 0.288793;
	 lookup_Cm_[1][12] = 0.282970;
	 lookup_Cm_[1][13] = 0.277621;
	 lookup_Cm_[1][14] = 0.272989;
	 lookup_Cm_[1][15] = 0.269167;
	 lookup_Cm_[1][16] = 0.265924;
	 lookup_Cm_[1][17] = 0.263224;
	 lookup_Cm_[1][18] = 0.261077;
	 lookup_Cm_[1][19] = 0.259496;
	 lookup_Cm_[1][20] = 0.258456;
	 lookup_Cm_[1][21] = 0.257922;
	 lookup_Cm_[1][22] = 0.257843;
	 lookup_Cm_[1][23] = 0.258159;
	 lookup_Cm_[1][24] = 0.258803;
	 lookup_Cm_[1][25] = 0.259718;
	 lookup_Cm_[1][26] = 0.260862;
	 lookup_Cm_[1][27] = 0.262177;
	 lookup_Cm_[1][28] = 0.263589;
	 lookup_Cm_[1][29] = 0.265013;
	 lookup_Cm_[1][30] = 0.266353;
	 lookup_Cm_[1][31] = 0.267489;
	 lookup_Cm_[1][32] = 0.268255;
	 lookup_Cm_[1][33] = 0.268376;
	 lookup_Cm_[1][34] = 0.267719;
	 lookup_Cm_[1][35] = 0.266753;
	 lookup_Cm_[1][36] = 0.265782;
	 lookup_Cm_[1][37] = 0.264861;
	 lookup_Cm_[1][38] = 0.263960;
	 lookup_Cm_[1][39] = 0.263071;
	 lookup_Cm_[1][40] = 0.262116;
	 lookup_Cm_[1][41] = 0.261090;
	 lookup_Cm_[1][42] = 0.259968;
	 lookup_Cm_[1][43] = 0.258726;
	 lookup_Cm_[1][44] = 0.257350;
	 lookup_Cm_[1][45] = 0.255798;
	 lookup_Cm_[1][46] = 0.253902;
	 lookup_Cm_[1][47] = 0.251470;
	 lookup_Cm_[1][48] = 0.248603;
	 lookup_Cm_[1][49] = 0.245828;
	 lookup_Cm_[1][50] = 0.242905;
	 lookup_Cm_[1][51] = 0.239802;
	 lookup_Cm_[1][52] = 0.236583;
	 lookup_Cm_[1][53] = 0.233181;
	 lookup_Cm_[1][54] = 0.229761;
	 lookup_Cm_[1][55] = 0.226352;
	 lookup_Cm_[1][56] = 0.222734;
	 lookup_Cm_[1][57] = 0.218866;
	 lookup_Cm_[1][58] = 0.214995;
	 lookup_Cm_[1][59] = 0.211443;
	 lookup_Cm_[1][60] = 0.207331;
	 lookup_Cm_[1][61] = 0.203114;
	 lookup_Cm_[1][62] = 0.197799;
	 lookup_Cm_[1][63] = 0.192399;
	 lookup_Cm_[1][64] = 0.109999;
	 lookup_Cm_[1][65] = 0.093602;
	 lookup_Cm_[1][66] = 0.081452;
	 lookup_Cm_[1][67] = 0.076001;
	 lookup_Cm_[1][68] = 0.067683;
	 lookup_Cm_[1][69] = 0.061923;
	 lookup_Cm_[1][70] = 0.056716;
	 lookup_Cm_[1][71] = 0.051469;
	 lookup_Cm_[1][72] = 0.045911;
	 lookup_Cm_[1][73] = 0.040734;
	 lookup_Cm_[1][74] = 0.035818;
	 lookup_Cm_[1][75] = 0.030814;
	 lookup_Cm_[1][76] = 0.024929;
	 lookup_Cm_[1][77] = 0.019102;
	 lookup_Cm_[1][78] = 0.012564;
	 lookup_Cm_[1][79] = 0.005535;
	 lookup_Cm_[1][80] = -0.001880;
	 lookup_Cm_[1][81] = -0.007643;
	 lookup_Cm_[1][82] = -0.012777;
	 lookup_Cm_[1][83] = -0.011422;
	 lookup_Cm_[1][84] = 0.009343;
	 lookup_Cm_[1][85] = 0.025460;
	 lookup_Cm_[1][86] = 0.030156;
	 lookup_Cm_[1][87] = 0.014673;
	 lookup_Cm_[1][88] = 0.001636;
	 lookup_Cm_[1][89] = -0.005309;
	 lookup_Cm_[1][90] = -0.027585;
	 lookup_Cm_[1][91] = -0.018704;
	 lookup_Cm_[1][92] = -0.020854;
	 lookup_Cm_[1][93] = -0.023985;
	 lookup_Cm_[1][94] = -0.022963;
	 lookup_Cm_[1][95] = -0.020217;
	 lookup_Cm_[1][96] = -0.016969;
	 lookup_Cm_[1][97] = -0.012566;
	 lookup_Cm_[1][98] = -0.006955;
	 lookup_Cm_[1][99] = 0.000845;
	 lookup_Cm_[1][100] = 0.012439;
	 lookup_Cm_[1][101] = 0.024513;
	 lookup_Cm_[1][102] = 0.040613;
	 lookup_Cm_[1][103] = 0.041383;
	 lookup_Cm_[1][104] = 0.038339;
	 lookup_Cm_[1][105] = 0.027659;
	 lookup_Cm_[1][106] = 0.023509;
	 lookup_Cm_[1][107] = 0.037558;
	 lookup_Cm_[1][108] = 0.037886;
	 lookup_Cm_[1][109] = 0.038053;
	 lookup_Cm_[1][110] = 0.028760;
	 lookup_Cm_[1][111] = 0.023171;
	 lookup_Cm_[1][112] = 0.019832;
	 lookup_Cm_[1][113] = 0.016694;
	 lookup_Cm_[1][114] = 0.012837;
	 lookup_Cm_[1][115] = 0.009903;
	 lookup_Cm_[1][116] = 0.006245;
	 lookup_Cm_[1][117] = 0.001812;
	 lookup_Cm_[1][118] = -0.003452;
	 lookup_Cm_[1][119] = -0.009046;
	 lookup_Cm_[1][120] = -0.014142;
	 lookup_Cm_[1][121] = -0.019475;
	 lookup_Cm_[1][122] = -0.024324;
	 lookup_Cm_[1][123] = -0.019658;
	 lookup_Cm_[1][124] = -0.020966;
	 lookup_Cm_[1][125] = -0.029163;
	 lookup_Cm_[1][126] = -0.058659;
	 lookup_Cm_[1][127] = -0.079753;
	 lookup_Cm_[1][128] = -0.094610;
	 lookup_Cm_[1][129] = -0.106491;
	 lookup_Cm_[1][130] = -0.116690;
	 lookup_Cm_[1][131] = -0.126226;
	 lookup_Cm_[1][132] = -0.135059;
	 lookup_Cm_[1][133] = -0.143410;
	 lookup_Cm_[1][134] = -0.151277;
	 lookup_Cm_[1][135] = -0.158686;
	 lookup_Cm_[1][136] = -0.165897;
	 lookup_Cm_[1][137] = -0.172411;
	 lookup_Cm_[1][138] = -0.178706;
	 lookup_Cm_[1][139] = -0.184560;
	 lookup_Cm_[1][140] = -0.190117;
	 lookup_Cm_[1][141] = -0.195287;
	 lookup_Cm_[1][142] = -0.200162;
	 lookup_Cm_[1][143] = -0.204648;
	 lookup_Cm_[1][144] = -0.208550;
	 lookup_Cm_[1][145] = -0.212237;
	 lookup_Cm_[1][146] = -0.215704;
	 lookup_Cm_[1][147] = -0.218915;
	 lookup_Cm_[1][148] = -0.221722;
	 lookup_Cm_[1][149] = -0.224249;
	 lookup_Cm_[1][150] = -0.226208;
	 lookup_Cm_[1][151] = -0.227435;
	 lookup_Cm_[1][152] = -0.227984;
	 lookup_Cm_[1][153] = -0.228153;
	 lookup_Cm_[1][154] = -0.228062;
	 lookup_Cm_[1][155] = -0.227423;
	 lookup_Cm_[1][156] = -0.226481;
	 lookup_Cm_[1][157] = -0.225285;
	 lookup_Cm_[1][158] = -0.223910;
	 lookup_Cm_[1][159] = -0.222406;
	 lookup_Cm_[1][160] = -0.220833;
	 lookup_Cm_[1][161] = -0.219272;
	 lookup_Cm_[1][162] = -0.217706;
	 lookup_Cm_[1][163] = -0.216176;
	 lookup_Cm_[1][164] = -0.214881;
	 lookup_Cm_[1][165] = -0.213810;
	 lookup_Cm_[1][166] = -0.213060;
	 lookup_Cm_[1][167] = -0.213172;
	 lookup_Cm_[1][168] = -0.213634;
	 lookup_Cm_[1][169] = -0.214761;
	 lookup_Cm_[1][170] = -0.217187;
	 lookup_Cm_[1][171] = -0.221786;
	 lookup_Cm_[1][172] = -0.227400;
	 lookup_Cm_[1][173] = -0.231152;
	 lookup_Cm_[1][174] = -0.235314;
	 lookup_Cm_[1][175] = -0.239209;
	 lookup_Cm_[1][176] = -0.242601;
	 lookup_Cm_[1][177] = -0.245415;
	 lookup_Cm_[1][178] = -0.246804;
	 lookup_Cm_[1][179] = -0.249053;
	 lookup_Cm_[1][180] = -0.256288;
	 lookup_Cm_[2][0] = 0.314449;
	 lookup_Cm_[2][1] = 0.309192;
	 lookup_Cm_[2][2] = 0.303935;
	 lookup_Cm_[2][3] = 0.298677;
	 lookup_Cm_[2][4] = 0.293420;
	 lookup_Cm_[2][5] = 0.288163;
	 lookup_Cm_[2][6] = 0.282905;
	 lookup_Cm_[2][7] = 0.277648;
	 lookup_Cm_[2][8] = 0.272391;
	 lookup_Cm_[2][9] = 0.267133;
	 lookup_Cm_[2][10] = 0.261876;
	 lookup_Cm_[2][11] = 0.256619;
	 lookup_Cm_[2][12] = 0.251361;
	 lookup_Cm_[2][13] = 0.246104;
	 lookup_Cm_[2][14] = 0.240847;
	 lookup_Cm_[2][15] = 0.235589;
	 lookup_Cm_[2][16] = 0.230332;
	 lookup_Cm_[2][17] = 0.225075;
	 lookup_Cm_[2][18] = 0.219817;
	 lookup_Cm_[2][19] = 0.214560;
	 lookup_Cm_[2][20] = 0.209303;
	 lookup_Cm_[2][21] = 0.204045;
	 lookup_Cm_[2][22] = 0.198788;
	 lookup_Cm_[2][23] = 0.193531;
	 lookup_Cm_[2][24] = 0.188273;
	 lookup_Cm_[2][25] = 0.183016;
	 lookup_Cm_[2][26] = 0.177759;
	 lookup_Cm_[2][27] = 0.172501;
	 lookup_Cm_[2][28] = 0.167244;
	 lookup_Cm_[2][29] = 0.161987;
	 lookup_Cm_[2][30] = 0.156729;
	 lookup_Cm_[2][31] = 0.151472;
	 lookup_Cm_[2][32] = 0.146215;
	 lookup_Cm_[2][33] = 0.140958;
	 lookup_Cm_[2][34] = 0.135700;
	 lookup_Cm_[2][35] = 0.130443;
	 lookup_Cm_[2][36] = 0.125186;
	 lookup_Cm_[2][37] = 0.119928;
	 lookup_Cm_[2][38] = 0.114671;
	 lookup_Cm_[2][39] = 0.109414;
	 lookup_Cm_[2][40] = 0.104156;
	 lookup_Cm_[2][41] = 0.098899;
	 lookup_Cm_[2][42] = 0.093642;
	 lookup_Cm_[2][43] = 0.088384;
	 lookup_Cm_[2][44] = 0.083127;
	 lookup_Cm_[2][45] = 0.077870;
	 lookup_Cm_[2][46] = 0.072612;
	 lookup_Cm_[2][47] = 0.067355;
	 lookup_Cm_[2][48] = 0.062098;
	 lookup_Cm_[2][49] = 0.056840;
	 lookup_Cm_[2][50] = 0.051583;
	 lookup_Cm_[2][51] = 0.046326;
	 lookup_Cm_[2][52] = 0.041068;
	 lookup_Cm_[2][53] = 0.035811;
	 lookup_Cm_[2][54] = 0.030554;
	 lookup_Cm_[2][55] = 0.025296;
	 lookup_Cm_[2][56] = 0.020614;
	 lookup_Cm_[2][57] = 0.015460;
	 lookup_Cm_[2][58] = 0.009682;
	 lookup_Cm_[2][59] = 0.004343;
	 lookup_Cm_[2][60] = -0.001315;
	 lookup_Cm_[2][61] = -0.006704;
	 lookup_Cm_[2][62] = -0.011754;
	 lookup_Cm_[2][63] = -0.016498;
	 lookup_Cm_[2][64] = -0.020656;
	 lookup_Cm_[2][65] = -0.024101;
	 lookup_Cm_[2][66] = -0.024155;
	 lookup_Cm_[2][67] = -0.022761;
	 lookup_Cm_[2][68] = -0.022700;
	 lookup_Cm_[2][69] = -0.023015;
	 lookup_Cm_[2][70] = -0.025472;
	 lookup_Cm_[2][71] = -0.036871;
	 lookup_Cm_[2][72] = -0.033551;
	 lookup_Cm_[2][73] = -0.028851;
	 lookup_Cm_[2][74] = -0.036921;
	 lookup_Cm_[2][75] = -0.033716;
	 lookup_Cm_[2][76] = -0.030866;
	 lookup_Cm_[2][77] = -0.025646;
	 lookup_Cm_[2][78] = -0.023874;
	 lookup_Cm_[2][79] = -0.022009;
	 lookup_Cm_[2][80] = -0.022246;
	 lookup_Cm_[2][81] = -0.021159;
	 lookup_Cm_[2][82] = -0.030052;
	 lookup_Cm_[2][83] = -0.030214;
	 lookup_Cm_[2][84] = -0.035442;
	 lookup_Cm_[2][85] = -0.034311;
	 lookup_Cm_[2][86] = -0.032455;
	 lookup_Cm_[2][87] = -0.042665;
	 lookup_Cm_[2][88] = -0.059412;
	 lookup_Cm_[2][89] = -0.069110;
	 lookup_Cm_[2][90] = -0.079815;
	 lookup_Cm_[2][91] = -0.088848;
	 lookup_Cm_[2][92] = -0.089186;
	 lookup_Cm_[2][93] = -0.084850;
	 lookup_Cm_[2][94] = -0.076240;
	 lookup_Cm_[2][95] = -0.065350;
	 lookup_Cm_[2][96] = -0.051916;
	 lookup_Cm_[2][97] = -0.037076;
	 lookup_Cm_[2][98] = -0.022444;
	 lookup_Cm_[2][99] = -0.010609;
	 lookup_Cm_[2][100] = -0.000528;
	 lookup_Cm_[2][101] = 0.001900;
	 lookup_Cm_[2][102] = -0.010137;
	 lookup_Cm_[2][103] = -0.034099;
	 lookup_Cm_[2][104] = -0.046096;
	 lookup_Cm_[2][105] = -0.052578;
	 lookup_Cm_[2][106] = -0.058107;
	 lookup_Cm_[2][107] = -0.063449;
	 lookup_Cm_[2][108] = -0.069107;
	 lookup_Cm_[2][109] = -0.072548;
	 lookup_Cm_[2][110] = -0.077501;
	 lookup_Cm_[2][111] = -0.079592;
	 lookup_Cm_[2][112] = -0.082544;
	 lookup_Cm_[2][113] = -0.085954;
	 lookup_Cm_[2][114] = -0.099614;
	 lookup_Cm_[2][115] = -0.177971;
	 lookup_Cm_[2][116] = -0.185153;
	 lookup_Cm_[2][117] = -0.191381;
	 lookup_Cm_[2][118] = -0.198025;
	 lookup_Cm_[2][119] = -0.203366;
	 lookup_Cm_[2][120] = -0.208314;
	 lookup_Cm_[2][121] = -0.212924;
	 lookup_Cm_[2][122] = -0.217231;
	 lookup_Cm_[2][123] = -0.221269;
	 lookup_Cm_[2][124] = -0.225059;
	 lookup_Cm_[2][125] = -0.228617;
	 lookup_Cm_[2][126] = -0.231959;
	 lookup_Cm_[2][127] = -0.235095;
	 lookup_Cm_[2][128] = -0.238036;
	 lookup_Cm_[2][129] = -0.240792;
	 lookup_Cm_[2][130] = -0.243371;
	 lookup_Cm_[2][131] = -0.245784;
	 lookup_Cm_[2][132] = -0.248037;
	 lookup_Cm_[2][133] = -0.250140;
	 lookup_Cm_[2][134] = -0.252102;
	 lookup_Cm_[2][135] = -0.253934;
	 lookup_Cm_[2][136] = -0.255644;
	 lookup_Cm_[2][137] = -0.257244;
	 lookup_Cm_[2][138] = -0.258746;
	 lookup_Cm_[2][139] = -0.260248;
	 lookup_Cm_[2][140] = -0.261750;
	 lookup_Cm_[2][141] = -0.263252;
	 lookup_Cm_[2][142] = -0.264754;
	 lookup_Cm_[2][143] = -0.266256;
	 lookup_Cm_[2][144] = -0.267758;
	 lookup_Cm_[2][145] = -0.269260;
	 lookup_Cm_[2][146] = -0.270762;
	 lookup_Cm_[2][147] = -0.272263;
	 lookup_Cm_[2][148] = -0.273765;
	 lookup_Cm_[2][149] = -0.275267;
	 lookup_Cm_[2][150] = -0.276769;
	 lookup_Cm_[2][151] = -0.278271;
	 lookup_Cm_[2][152] = -0.279773;
	 lookup_Cm_[2][153] = -0.281275;
	 lookup_Cm_[2][154] = -0.282777;
	 lookup_Cm_[2][155] = -0.284279;
	 lookup_Cm_[2][156] = -0.285781;
	 lookup_Cm_[2][157] = -0.287282;
	 lookup_Cm_[2][158] = -0.288784;
	 lookup_Cm_[2][159] = -0.290286;
	 lookup_Cm_[2][160] = -0.291788;
	 lookup_Cm_[2][161] = -0.293290;
	 lookup_Cm_[2][162] = -0.294792;
	 lookup_Cm_[2][163] = -0.296294;
	 lookup_Cm_[2][164] = -0.297796;
	 lookup_Cm_[2][165] = -0.299298;
	 lookup_Cm_[2][166] = -0.300800;
	 lookup_Cm_[2][167] = -0.302302;
	 lookup_Cm_[2][168] = -0.303803;
	 lookup_Cm_[2][169] = -0.305305;
	 lookup_Cm_[2][170] = -0.306807;
	 lookup_Cm_[2][171] = -0.308309;
	 lookup_Cm_[2][172] = -0.309811;
	 lookup_Cm_[2][173] = -0.311313;
	 lookup_Cm_[2][174] = -0.312815;
	 lookup_Cm_[2][175] = -0.314317;
	 lookup_Cm_[2][176] = -0.315819;
	 lookup_Cm_[2][177] = -0.317321;
	 lookup_Cm_[2][178] = -0.318822;
	 lookup_Cm_[2][179] = -0.320324;
	 lookup_Cm_[2][180] = -0.321826;
	 lookup_Cm_[3][0] = 0.174037;
	 lookup_Cm_[3][1] = 0.170955;
	 lookup_Cm_[3][2] = 0.167872;
	 lookup_Cm_[3][3] = 0.164790;
	 lookup_Cm_[3][4] = 0.161707;
	 lookup_Cm_[3][5] = 0.158625;
	 lookup_Cm_[3][6] = 0.155542;
	 lookup_Cm_[3][7] = 0.147574;
	 lookup_Cm_[3][8] = 0.138872;
	 lookup_Cm_[3][9] = 0.129799;
	 lookup_Cm_[3][10] = 0.120489;
	 lookup_Cm_[3][11] = 0.111000;
	 lookup_Cm_[3][12] = 0.101320;
	 lookup_Cm_[3][13] = 0.091721;
	 lookup_Cm_[3][14] = 0.082017;
	 lookup_Cm_[3][15] = 0.071889;
	 lookup_Cm_[3][16] = 0.061780;
	 lookup_Cm_[3][17] = 0.052240;
	 lookup_Cm_[3][18] = 0.043713;
	 lookup_Cm_[3][19] = 0.035670;
	 lookup_Cm_[3][20] = 0.027003;
	 lookup_Cm_[3][21] = 0.019387;
	 lookup_Cm_[3][22] = 0.013560;
	 lookup_Cm_[3][23] = 0.027746;
	 lookup_Cm_[3][24] = 0.031946;
	 lookup_Cm_[3][25] = 0.033413;
	 lookup_Cm_[3][26] = 0.045719;
	 lookup_Cm_[3][27] = 0.042774;
	 lookup_Cm_[3][28] = 0.039258;
	 lookup_Cm_[3][29] = 0.035553;
	 lookup_Cm_[3][30] = 0.034701;
	 lookup_Cm_[3][31] = 0.034374;
	 lookup_Cm_[3][32] = 0.032403;
	 lookup_Cm_[3][33] = 0.031358;
	 lookup_Cm_[3][34] = 0.029154;
	 lookup_Cm_[3][35] = 0.026805;
	 lookup_Cm_[3][36] = 0.024451;
	 lookup_Cm_[3][37] = 0.022808;
	 lookup_Cm_[3][38] = 0.021260;
	 lookup_Cm_[3][39] = 0.019497;
	 lookup_Cm_[3][40] = 0.017191;
	 lookup_Cm_[3][41] = 0.015032;
	 lookup_Cm_[3][42] = 0.012845;
	 lookup_Cm_[3][43] = 0.010687;
	 lookup_Cm_[3][44] = 0.008638;
	 lookup_Cm_[3][45] = 0.007067;
	 lookup_Cm_[3][46] = 0.003593;
	 lookup_Cm_[3][47] = 0.003849;
	 lookup_Cm_[3][48] = 0.002065;
	 lookup_Cm_[3][49] = 0.000344;
	 lookup_Cm_[3][50] = -0.001969;
	 lookup_Cm_[3][51] = -0.004118;
	 lookup_Cm_[3][52] = -0.006473;
	 lookup_Cm_[3][53] = -0.008333;
	 lookup_Cm_[3][54] = -0.010111;
	 lookup_Cm_[3][55] = -0.013391;
	 lookup_Cm_[3][56] = -0.016474;
	 lookup_Cm_[3][57] = -0.018484;
	 lookup_Cm_[3][58] = -0.018571;
	 lookup_Cm_[3][59] = -0.020654;
	 lookup_Cm_[3][60] = -0.024086;
	 lookup_Cm_[3][61] = -0.025344;
	 lookup_Cm_[3][62] = -0.026478;
	 lookup_Cm_[3][63] = -0.028190;
	 lookup_Cm_[3][64] = -0.029855;
	 lookup_Cm_[3][65] = -0.031424;
	 lookup_Cm_[3][66] = -0.033817;
	 lookup_Cm_[3][67] = -0.036903;
	 lookup_Cm_[3][68] = -0.038501;
	 lookup_Cm_[3][69] = -0.037059;
	 lookup_Cm_[3][70] = -0.038630;
	 lookup_Cm_[3][71] = -0.037137;
	 lookup_Cm_[3][72] = -0.037781;
	 lookup_Cm_[3][73] = -0.038272;
	 lookup_Cm_[3][74] = -0.038223;
	 lookup_Cm_[3][75] = -0.037762;
	 lookup_Cm_[3][76] = -0.037547;
	 lookup_Cm_[3][77] = -0.036383;
	 lookup_Cm_[3][78] = -0.034978;
	 lookup_Cm_[3][79] = -0.112967;
	 lookup_Cm_[3][80] = -0.135863;
	 lookup_Cm_[3][81] = -0.135830;
	 lookup_Cm_[3][82] = -0.146584;
	 lookup_Cm_[3][83] = -0.147630;
	 lookup_Cm_[3][84] = -0.146182;
	 lookup_Cm_[3][85] = -0.139470;
	 lookup_Cm_[3][86] = -0.134772;
	 lookup_Cm_[3][87] = -0.133262;
	 lookup_Cm_[3][88] = -0.131189;
	 lookup_Cm_[3][89] = -0.132579;
	 lookup_Cm_[3][90] = -0.132476;
	 lookup_Cm_[3][91] = -0.128422;
	 lookup_Cm_[3][92] = -0.124019;
	 lookup_Cm_[3][93] = -0.114548;
	 lookup_Cm_[3][94] = -0.101118;
	 lookup_Cm_[3][95] = -0.093531;
	 lookup_Cm_[3][96] = -0.085945;
	 lookup_Cm_[3][97] = -0.078359;
	 lookup_Cm_[3][98] = -0.070773;
	 lookup_Cm_[3][99] = -0.098664;
	 lookup_Cm_[3][100] = -0.118053;
	 lookup_Cm_[3][101] = -0.126526;
	 lookup_Cm_[3][102] = -0.152011;
	 lookup_Cm_[3][103] = -0.159638;
	 lookup_Cm_[3][104] = -0.167772;
	 lookup_Cm_[3][105] = -0.174363;
	 lookup_Cm_[3][106] = -0.198239;
	 lookup_Cm_[3][107] = -0.203967;
	 lookup_Cm_[3][108] = -0.205685;
	 lookup_Cm_[3][109] = -0.207404;
	 lookup_Cm_[3][110] = -0.209122;
	 lookup_Cm_[3][111] = -0.210841;
	 lookup_Cm_[3][112] = -0.212560;
	 lookup_Cm_[3][113] = -0.214278;
	 lookup_Cm_[3][114] = -0.215997;
	 lookup_Cm_[3][115] = -0.217715;
	 lookup_Cm_[3][116] = -0.219434;
	 lookup_Cm_[3][117] = -0.221153;
	 lookup_Cm_[3][118] = -0.222871;
	 lookup_Cm_[3][119] = -0.224590;
	 lookup_Cm_[3][120] = -0.226308;
	 lookup_Cm_[3][121] = -0.228027;
	 lookup_Cm_[3][122] = -0.229746;
	 lookup_Cm_[3][123] = -0.231464;
	 lookup_Cm_[3][124] = -0.233183;
	 lookup_Cm_[3][125] = -0.234901;
	 lookup_Cm_[3][126] = -0.236620;
	 lookup_Cm_[3][127] = -0.238339;
	 lookup_Cm_[3][128] = -0.240057;
	 lookup_Cm_[3][129] = -0.241776;
	 lookup_Cm_[3][130] = -0.243494;
	 lookup_Cm_[3][131] = -0.245213;
	 lookup_Cm_[3][132] = -0.246932;
	 lookup_Cm_[3][133] = -0.248650;
	 lookup_Cm_[3][134] = -0.250369;
	 lookup_Cm_[3][135] = -0.252087;
	 lookup_Cm_[3][136] = -0.253806;
	 lookup_Cm_[3][137] = -0.255525;
	 lookup_Cm_[3][138] = -0.257243;
	 lookup_Cm_[3][139] = -0.258962;
	 lookup_Cm_[3][140] = -0.260680;
	 lookup_Cm_[3][141] = -0.262399;
	 lookup_Cm_[3][142] = -0.264118;
	 lookup_Cm_[3][143] = -0.265836;
	 lookup_Cm_[3][144] = -0.267555;
	 lookup_Cm_[3][145] = -0.269273;
	 lookup_Cm_[3][146] = -0.270992;
	 lookup_Cm_[3][147] = -0.272711;
	 lookup_Cm_[3][148] = -0.274429;
	 lookup_Cm_[3][149] = -0.276148;
	 lookup_Cm_[3][150] = -0.277866;
	 lookup_Cm_[3][151] = -0.279585;
	 lookup_Cm_[3][152] = -0.281304;
	 lookup_Cm_[3][153] = -0.283022;
	 lookup_Cm_[3][154] = -0.284741;
	 lookup_Cm_[3][155] = -0.286459;
	 lookup_Cm_[3][156] = -0.288178;
	 lookup_Cm_[3][157] = -0.289897;
	 lookup_Cm_[3][158] = -0.291615;
	 lookup_Cm_[3][159] = -0.293334;
	 lookup_Cm_[3][160] = -0.295052;
	 lookup_Cm_[3][161] = -0.296771;
	 lookup_Cm_[3][162] = -0.298490;
	 lookup_Cm_[3][163] = -0.300208;
	 lookup_Cm_[3][164] = -0.301927;
	 lookup_Cm_[3][165] = -0.303645;
	 lookup_Cm_[3][166] = -0.305364;
	 lookup_Cm_[3][167] = -0.307083;
	 lookup_Cm_[3][168] = -0.308801;
	 lookup_Cm_[3][169] = -0.310520;
	 lookup_Cm_[3][170] = -0.312238;
	 lookup_Cm_[3][171] = -0.313957;
	 lookup_Cm_[3][172] = -0.315676;
	 lookup_Cm_[3][173] = -0.317394;
	 lookup_Cm_[3][174] = -0.319113;
	 lookup_Cm_[3][175] = -0.320831;
	 lookup_Cm_[3][176] = -0.322550;
	 lookup_Cm_[3][177] = -0.324269;
	 lookup_Cm_[3][178] = -0.325987;
	 lookup_Cm_[3][179] = -0.327706;
	 lookup_Cm_[3][180] = -0.329424;
}
