/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file dubin.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Vector field navigation using Dubin's path
 *
 ******************************************************************************/

#include "dubin.h"
#include "print_util.h"
#include "delay.h"

/**
 * \brief 		Computes the arc length between two point of a circle
 * 				 
 * \param 		p1 				The coordinates of the first point
 * \param 		p2				The coordinates of the second point
 * \param 		c				The coordinates of the center of the circle
 * \param 		sense 			The sense of rotation around the circle, Positive: clockwise
 *
 * \return 		The arc length
 */
static float dubin_arc_length_2d(const float p1[3], const float p2[3], const float c[3], const int8_t sense);

/**
 * \brief 		Finds the tangent point for two circles, rotating in the sense of the sign of r1 and r2 respectively 
 * 				 
 * \param 		t1 				The coordinates of the tangent point to the first circle
 * \param 		t2				The coordinates of the tangent point to the second circle
 * \param 		c1				The coordinates of the center of the first circle
 * \param 		c2				The coordinates of the center of the second circle
 * \param 		r1 				The radius of the first circle (Positive: clockwise)
 * \param 		r2 				The radius of the second circle (Positive: clockwise)
 */
static void dubin_find_tangent(float t1[3], float t2[3], const float c1[3], const float c2[3], const float r1, const float r2);

/**
 * \brief 		Computes the Dubin path length between two points
 * 				 
 * \param 		t1 				The coordinates of the tangent point to the first circle
 * \param 		t2				The coordinates of the tangent point to the second circle
 * \param 		c1				The coordinates of the center of the first circle
 * \param 		c2				The coordinates of the center of the second circle
 * \param 		wp1 			The starting point
 * \param 		wp2 			The ending point
 * \param 		sense1 			The sense of rotation around the first circle (Positive: clockwise)
 * \param 		sense2 			The sense of rotation around the second circle (Positive: clockwise)
 *
 * \return 		The Dubin's path length
 */
static float dubin_path_length(float t1[3], float t2[3], const float c1[3], const float c2[3], const float wp1[3], const float wp2[3], const int8_t sense1, const int8_t sense2);


void dubin_line(float tvel[3], const float line_dir[3], const float line_origin[3], const float pos[3], const float speed, const float one_over_scaling)
{
	//parameters
	//float one_over_scaling=0.1; //defines the main influence area [m^-1]
	
	float e_t[3], e_r[3]; //tangential and radial unit vectors
	float v_t_norm, v_r_norm;
	float op[3], rad[3], normal_dist, projection_length, k_r;
	
	uint32_t i;

	if(vectors_norm_sqr(line_dir) > 0.0f)
	{
		//project pos on line ->v_t
		for (i = 0; i < 2; ++i)
		{
			op[i] = pos[i] - line_origin[i];
		}
		op[Z] = 0.0f;

		vectors_normalize(line_dir, e_t);//get tangent direction
		
		projection_length = vectors_scalar_product(e_t,op);
		
		for (i = 0; i < 2; ++i)
		{
			rad[i] = e_t[i] * projection_length - op[i];
		}
		rad[Z] = 0.0f;
		
		normal_dist = vectors_norm(rad);

		if(normal_dist > 0.0f)
		{
			vectors_normalize(rad,e_r);//get radial direction
		}
		else
		{
			e_r[0]=0;
			e_r[1]=0;
			e_r[2]=0;
		}
		
		k_r = 2.0f / PI * atan(normal_dist*one_over_scaling); //map all possible distances to 0 to 1;
		v_r_norm = sqrtf(1-k_r*k_r);

		for (i = 0; i < 2; ++i)
		{
			tvel[i] = e_t[i]*v_r_norm + e_r[i]*k_r;
		}
		tvel[Z] = 0.0f;

		v_t_norm = vectors_norm(tvel);
		for (i = 0; i < 2; ++i)
		{
			tvel[i] *= speed / v_t_norm;
		}
		tvel[Z] = 0.0f;

	}
}

void dubin_circle(float tvel[3], const float circle[3], float radius_mavlink, const float pos[3], float speed, float one_over_scaling)
{
	float radius = -radius_mavlink;

	float tan_dir[3], tan_origin[3];
	float rel_pos_norm[3];

	uint32_t i;

	float rel_pos[3];

	for (i = 0; i < 2; ++i)
	{
		rel_pos[i] = circle[i] - pos[i];
	}
	rel_pos[Z] = 0.0f;

	if (vectors_norm_sqr(rel_pos) > 0.0f)
	{

		rel_pos_norm[X] = rel_pos[X];
		rel_pos_norm[Y] = rel_pos[Y];
		rel_pos_norm[Z] = 0.0f;

		vectors_normalize(rel_pos_norm, rel_pos_norm);

		tan_dir[X] = -rel_pos_norm[Y] * maths_sign(radius);
		tan_dir[Y] = rel_pos_norm[X] * maths_sign(radius);
		tan_dir[Z] = 0.0f;

		for (i = 0; i < 2; ++i)
		{
			tan_origin[i] = circle[i] - maths_f_abs(radius) * rel_pos_norm[i];
		}
		tan_origin[Z] = 0.0f;

		//compute a vectorfield using the tangent.
		dubin_line(tvel, tan_dir, tan_origin, pos, speed, one_over_scaling);
	}
}

static float dubin_arc_length_2d(const float p1[3], const float p2[3], const float c[3], const int8_t sense)
{	
	uint32_t i;
	float v1[3];
	float v2[3];

	for (i = 0; i < 2; ++i)
	{
		v1[i] = p1[i] - c[i];
		v2[i] = p2[i] - c[i];
	}
	v1[2] = 0.0f;
	v2[2] = 0.0f;

	float v1_normalized[3];
	float v2_normalized[3];

	vectors_normalize(v1,v1_normalized);
	vectors_normalize(v2,v2_normalized);

	float angle = acos(vectors_scalar_product(v1_normalized,v2_normalized));

	int32_t sense_of_measure = v1[0]*v2[1] - v1[1]*v2[0];

	float result;
	if(sense_of_measure*sense < 0) //if direction of angle=direction of flight
	{
		result = angle*vectors_norm(v1);
	}
	else
	{
		result = (2*PI-angle)*vectors_norm(v1);
	}

	return result;
}

static void dubin_find_tangent(float t1[3], float t2[3], const float c1[3], const float c2[3], const float r1, const float r2)
{
	// t1, t2: tangent points, the sense of the circles c1 and c2 is defined by the sign of the radius r1 and r2
	// Positive value for clockwise orbit

	uint32_t i;

	float u[3], v[3];
	for (i = 0; i < 2; ++i)
	{
		v[i] = c2[i]-c1[i];
	}
	v[i] = 0.0f;

	float dist = vectors_norm(v);
	vectors_normalize(v,u);

	float alpha = atan2(u[1],u[0]);

	float cos_alpha = cos(alpha);
	float sin_alpha = sin(alpha);

	float tan_normal[3];
	tan_normal[2] = 0.0f;

	tan_normal[0] = (r1-r2)/dist;
	tan_normal[1] = sqrtf(SQR(dist)-SQR(r1-r2))/dist;

	float tt1[3];
	float tt2[3];

	for (i = 0; i < 2; ++i)
	{
		tt1[i] = r1 * tan_normal[i];
		tt2[i] = r2 * tan_normal[i];
	}
	tt1[2] = 0.0f;
	tt2[2] = 0.0f;

	t1[0] = c1[0] + cos_alpha * tt1[0] - sin_alpha * tt1[1];
	t1[1] = c1[1] + sin_alpha * tt1[0] + cos_alpha * tt1[1];
	t1[2] = tt1[2];

	t2[0] = c2[0] + cos_alpha * tt2[0] - sin_alpha * tt2[1];
	t2[1] = c2[1] + sin_alpha * tt2[0] + cos_alpha * tt2[1];
	t2[2] = tt2[2];

}

static float dubin_path_length(float t1[3], float t2[3], const float c1[3], const float c2[3], const float wp1[3], const float wp2[3], const int8_t sense1, const int8_t sense2)
{
	uint32_t i;
	float w1[3];
	float w2[3];

	for (i = 0; i < 2; ++i)
	{
		w1[i] = wp1[i] - c1[i];
		w2[i] = wp2[i] - c2[i];
	}
	w1[Z] = 0.0f;
	w2[Z] = 0.0f;

	float norm_w1 = sense1*vectors_norm(w1);
	float norm_w2 = sense2*vectors_norm(w2);

	dubin_find_tangent(t1, t2, c1, c2, norm_w1, norm_w2);

	float t1_t2[3];
	for (i = 0; i < 2; ++i)
	{
		t1_t2[i] = t1[i] - t2[i];
	}
	t1_t2[Z] = 0.0f;

	float t1_t2_norm = vectors_norm(t1_t2);

	float arc_1 = dubin_arc_length_2d(wp1,t1,c1,sense1);
	float arc_2 = dubin_arc_length_2d(t2,wp2,c2,sense2);

	return t1_t2_norm + arc_1 + arc_2;
}

// void dubin_test_code(void)
// {
// 	delay_ms(2000);

// 	float wp1[3], wp2[3], d1[3], d2[3], sense_2;

// 	wp1[X] = -66.676f;
// 	wp1[Y] = 41.777f;
// 	wp1[Z] = -30.004f;

// 	wp2[X] = 72.576f;
// 	wp2[Y] = -5.029f;
// 	wp2[Z] = 0.0f;

// 	d1[X] = -38.436f;
// 	d1[Y] = -11.071;
// 	d1[Z] = 0.0f;

// 	d2[X] = -9.558f;
// 	d2[Y] = -28.436f;
// 	d2[Z] = 0.0f;

// 	sense_2 = -1.0;

// 	dubin_2d(wp1, wp2, d1, d2, sense_2);

// 	delay_ms(1000);
// }

dubin_t dubin_2d(const float wp1[3], const float wp2[3], const float d1[3], const float d2[3], float sense_mavlink)
{
	float sense_2 = -sense_mavlink;

	dubin_t out,temp;
	
	out.length=0;

	uint32_t i,j;

	//directions already turned by 90 degrees
	float r1[3];
	r1[X] = -d1[Y];
	r1[Y] = d1[X];
	r1[Z] = 0.0f;

	float r2[3];
	r2[X] = -d2[Y];
	r2[Y] = d2[X];
	r2[Z] = 0.0f;

	float rad1 = vectors_norm(r1);
	float rad2 = vectors_norm(r2);

	float dist;

	bool init = false;

	// Init 
	for (i = 0; i < 3; ++i)
	{
		out.circle_center_1[i] = wp1[i];
		out.circle_center_2[i] = wp2[i];
		out.tangent_point_1[i] = wp1[i] - r1[i];
		out.tangent_point_2[i] = wp2[i] - sense_2*r2[i];
	}

	float c1_c2[3];
	//find shortest path for all feasible solutions
	for (i=0; i<2; i++)
	{
		temp.sense_1 = i&1?1:-1; 
		
		//temp.sense_2 = i&2?1:-1; 
		
		for (j = 0; j < 2; j++)
		{
			temp.circle_center_1[j] = wp1[j] - temp.sense_1*r1[j];
			temp.circle_center_2[j] = wp2[j] - sense_2*r2[j];

			c1_c2[j] = temp.circle_center_1[j] - temp.circle_center_2[j];
		}
		temp.circle_center_1[2] = 0.0f;
		temp.circle_center_2[2] = 0.0f;
		c1_c2[Z] = 0.0f;

		dist = vectors_norm(c1_c2);

		//check feasibility
		if( ( (rad1+rad2) < dist) || ( ((dist+rad1) > rad2) && ((dist+rad2) > rad1) && (temp.sense_1*sense_2==1) ) )
		{
			temp.length = dubin_path_length(	temp.tangent_point_1,
												temp.tangent_point_2,
												temp.circle_center_1,
												temp.circle_center_2,
												wp1,
												wp2,
												temp.sense_1,
												sense_2);
			
			for (j = 0; j < 2; j++)
			{
				temp.line_direction[j] = temp.tangent_point_2[j] - temp.tangent_point_1[j];
			}
			temp.line_direction[Z] = 0.0f;

			if ( (!init) || ( temp.length < out.length) ) //save temp if the pathlength is shorter, or if this is the first run
			{
				init = true;
				out=temp;
			}
		}
		else
		{
			print_util_dbg_print("This sense combination is not possible\r\n");
		}
	}

	if (!init)
	{
		print_util_dbg_print("No possible solutions...\r\n");
	}

	return out;
}