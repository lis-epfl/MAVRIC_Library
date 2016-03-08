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
 * \file ahrs_ekf.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief EKF attitude estimation
 *
 ******************************************************************************/


#include "matrix.hpp"
#include "ahrs_ekf.h"

extern "C"
{
#include "constants.h"
#include "print_util.h"
#include "maths.h"
#include "vectors.h"
#include "quaternions.h"
#include "delay.h"
}


using namespace mat;

Mat<7,1> x_state;
Mat<7,7> F;
Mat<7,7> P;
Mat<7,7> Q;
Mat<3,3> R_acc;
Mat<3,3> R_mag;
Mat<7,7> Id;

void ahrs_ekf_init_cpp(ahrs_ekf_t* ahrs_ekf);

void ahrs_ekf_predict_step(ahrs_ekf_t* ahrs_ekf);

void ahrs_ekf_update_step_acc(ahrs_ekf_t* ahrs_ekf);
void ahrs_ekf_update_step_mag(ahrs_ekf_t* ahrs_ekf);

void ahrs_ekf_init_cpp(ahrs_ekf_t* ahrs_ekf)
{
	P = Mat<7,7>(10.0f,true);

	P(3,3) = 10.0f;
	P(4,4) = 10.0f;
	P(5,5) = 10.0f;
	P(6,6) = 10.0f;

	// Initalisation of the state

	x_state(0,0) = 0.0f;
	x_state(1,0) = 0.0f;
	x_state(2,0) = 0.0f;
	x_state(3,0) = 1.0f;
	x_state(4,0) = 0.0f;
	x_state(5,0) = 0.0f;
	x_state(6,0) = 0.0f;

	R_acc(0,0) = ahrs_ekf->config.R_acc;//0.02f;
	R_acc(1,1) = ahrs_ekf->config.R_acc;//0.02f;
	R_acc(2,2) = ahrs_ekf->config.R_acc;//0.02f;

	R_mag(0,0) = ahrs_ekf->config.R_mag; //0.05f;
	R_mag(1,1) = ahrs_ekf->config.R_mag; //0.05f;
	R_mag(2,2) = ahrs_ekf->config.R_mag; //0.05f;

	Id = Mat<7,7>(1.0f,true);
}

void ahrs_ekf_predict_step(ahrs_ekf_t* ahrs_ekf)
{
	float dt = ahrs_ekf->ahrs->dt;
	
	float dt2_2 = dt * dt * 0.5f;
	float dt3_3 = dt * dt * dt / 3.0f;

	float w_x = ahrs_ekf->imu->scaled_gyro.data[X];
	float w_y = ahrs_ekf->imu->scaled_gyro.data[Y];
	float w_z = ahrs_ekf->imu->scaled_gyro.data[Z];

	float w_sqr = w_x*w_x + w_y*w_y + w_z*w_z;

	Mat<7,1> x_k1k1 = x_state;

	Mat<7,1> x_kk1;
	// x(k,k-1) = f(x(k-1,k-1),u(k)); // with zero-order Taylor expansion
	x_kk1(0,0) = x_k1k1(0,0);
	x_kk1(1,0) = x_k1k1(1,0);
	x_kk1(2,0) = x_k1k1(2,0);
	/*x_kk1(3,0) = x_k1k1(3,0) + 0.5f* (-(w_x-x_k1k1(0,0))*x_k1k1(4,0) - (w_y-x_k1k1(1,0))*x_k1k1(5,0) - (w_z-x_k1k1(2,0))*x_k1k1(6,0)) * dt;
	x_kk1(4,0) = x_k1k1(4,0) + 0.5f* ((w_x-x_k1k1(0,0))*x_k1k1(3,0) - (w_z-x_k1k1(2,0))*x_k1k1(5,0) + (w_y-x_k1k1(1,0))*x_k1k1(6,0)) * dt;
	x_kk1(5,0) = x_k1k1(5,0) + 0.5f* ((w_y-x_k1k1(1,0))*x_k1k1(3,0) + (w_z-x_k1k1(2,0))*x_k1k1(4,0) - (w_x-x_k1k1(0,0))*x_k1k1(6,0)) * dt;
	x_kk1(6,0) = x_k1k1(6,0) + 0.5f* ((w_z-x_k1k1(2,0))*x_k1k1(3,0) - (w_y-x_k1k1(1,0))*x_k1k1(4,0) + (w_x-x_k1k1(0,0))*x_k1k1(5,0)) * dt;*/

	x_kk1(3,0) = x_k1k1(3,0) + 0.5f* (-(w_x-x_k1k1(0,0))*x_k1k1(4,0) - (w_y-x_k1k1(1,0))*x_k1k1(5,0) - (w_z-x_k1k1(2,0))*x_k1k1(6,0)) * dt;
	x_kk1(4,0) = x_k1k1(4,0) + 0.5f* ((w_x-x_k1k1(0,0))*x_k1k1(3,0) + (w_z-x_k1k1(2,0))*x_k1k1(5,0) - (w_y-x_k1k1(1,0))*x_k1k1(6,0)) * dt;
	x_kk1(5,0) = x_k1k1(5,0) + 0.5f* ((w_y-x_k1k1(1,0))*x_k1k1(3,0) - (w_z-x_k1k1(2,0))*x_k1k1(4,0) + (w_x-x_k1k1(0,0))*x_k1k1(6,0)) * dt;
	x_kk1(6,0) = x_k1k1(6,0) + 0.5f* ((w_z-x_k1k1(2,0))*x_k1k1(3,0) + (w_y-x_k1k1(1,0))*x_k1k1(4,0) - (w_x-x_k1k1(0,0))*x_k1k1(5,0)) * dt;

	// F(k,k-1) = I + jacobian(x(k-1),u(k))*dt;
	F(0,0) = 1.0f;
	F(1,1) = 1.0f;
	F(2,2) = 1.0f;

	/*F(3,0) = x_k1k1(4,0) * dt;
	F(3,1) = x_k1k1(5,0) * dt;
	F(3,2) = x_k1k1(6,0) * dt;
	F(3,3) = 1.0f; // 1.0f + 0.0f;
	F(3,4) = -(w_x-x_k1k1(0,0)) * dt;
	F(3,5) = -(w_y-x_k1k1(1,0)) * dt;
	F(3,6) = -(w_z-x_k1k1(2,0)) * dt;
	
	F(4,0) = -x_k1k1(3,0) * dt;
	F(4,1) = -x_k1k1(6,0) * dt;
	F(4,2) = x_k1k1(5,0) * dt;
	F(4,3) = (w_x-x_k1k1(0,0)) * dt;
	F(4,4) = 1.0f; // 1.0f + 0.0f;
	F(4,5) = -(w_z-x_k1k1(2,0)) * dt;
	F(4,6) = (w_y-x_k1k1(1,0)) * dt;
	
	F(5,0) = x_k1k1(6,0) * dt;
	F(5,1) = -x_k1k1(3,0) * dt;
	F(5,2) = -x_k1k1(4,0) * dt;
	F(5,3) = (w_y-x_k1k1(1,0)) * dt;
	F(5,4) = (w_z-x_k1k1(2,0)) * dt;
	F(5,5) = 1.0f; // 1.0f + 0.0f;
	F(5,6) = -(w_x-x_k1k1(0,0)) * dt;
	
	F(6,0) = -x_k1k1(5,0) * dt;
	F(6,1) = x_k1k1(4,0) * dt;
	F(6,2) = -x_k1k1(3,0) * dt;
	F(6,3) = (w_z-x_k1k1(2,0)) * dt;
	F(6,4) = -(w_y-x_k1k1(1,0)) * dt;
	F(6,5) = (w_x-x_k1k1(0,0)) * dt;
	F(6,6) = 1.0f; // 1.0f + 0.0f;*/


	F(3,0) = x_k1k1(4,0) * dt;
	F(3,1) = x_k1k1(5,0) * dt;
	F(3,2) = x_k1k1(6,0) * dt;
	F(3,3) = 1.0f; // 1.0f + 0.0f;
	F(3,4) = -(w_x-x_k1k1(0,0)) * dt;
	F(3,5) = -(w_y-x_k1k1(1,0)) * dt;
	F(3,6) = -(w_z-x_k1k1(2,0)) * dt;
	
	F(4,0) = -x_k1k1(3,0) * dt;
	F(4,1) = x_k1k1(6,0) * dt;
	F(4,2) = -x_k1k1(5,0) * dt;
	F(4,3) = (w_x-x_k1k1(0,0)) * dt;
	F(4,4) = 1.0f; // 1.0f + 0.0f;
	F(4,5) = (w_z-x_k1k1(2,0)) * dt;
	F(4,6) = -(w_y-x_k1k1(1,0)) * dt;
	
	F(5,0) = -x_k1k1(6,0) * dt;
	F(5,1) = -x_k1k1(3,0) * dt;
	F(5,2) = x_k1k1(4,0) * dt;
	F(5,3) = (w_y-x_k1k1(1,0)) * dt;
	F(5,4) = -(w_z-x_k1k1(2,0)) * dt;
	F(5,5) = 1.0f; // 1.0f + 0.0f;
	F(5,6) = (w_x-x_k1k1(0,0)) * dt;
	
	F(6,0) = x_k1k1(5,0) * dt;
	F(6,1) = -x_k1k1(4,0) * dt;
	F(6,2) = -x_k1k1(3,0) * dt;
	F(6,3) = (w_z-x_k1k1(2,0)) * dt;
	F(6,4) = (w_y-x_k1k1(1,0)) * dt;
	F(6,5) = -(w_x-x_k1k1(0,0)) * dt;
	F(6,6) = 1.0f; // 1.0f + 0.0f;

	// Q(k) = cov(del_w * del_w^T)
	
	Q(0,0) = ahrs_ekf->config.sigma_w_sqr * dt;
	Q(0,1) = 0.0f;
	Q(0,2) = 0.0f;
	Q(0,3) = x_kk1[4] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(0,4) = -x_kk1[3] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(0,5) = -x_kk1[6] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(0,6) = x_kk1[5] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;

	Q(1,0) = 0.0f;
	Q(1,1) = ahrs_ekf->config.sigma_w_sqr * dt;
	Q(1,2) = 0.0f;
	Q(1,3) = x_kk1[5] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(1,4) = x_kk1[6] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(1,5) = -x_kk1[3] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(1,6) = -x_kk1[4] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;

	Q(2,0) = 0.0f;
	Q(2,1) = 0.0f;
	Q(2,2) = ahrs_ekf->config.sigma_w_sqr * dt;
	Q(2,3) = x_kk1[6] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(2,4) = -x_kk1[5] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(2,5) = x_kk1[4] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(2,6) = -x_kk1[3] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;

	Q(3,0) = x_kk1[4] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(3,1) = x_kk1[5] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(3,2) = x_kk1[6] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(3,3) = ahrs_ekf->config.sigma_r_sqr * dt + w_sqr * ahrs_ekf->config.sigma_r_sqr * dt3_3 + (x_kk1[4]*x_kk1[4]+x_kk1[5]*x_kk1[5]+x_kk1[6]*x_kk1[6])*ahrs_ekf->config.sigma_w_sqr*dt3_3;
	Q(3,4) = -x_kk1[3] * x_kk1[4] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;
	Q(3,5) = -x_kk1[3] * x_kk1[5] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;
	Q(3,6) = -x_kk1[3] * x_kk1[6] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;

	Q(4,0) = -x_kk1[3] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(4,1) = x_kk1[6] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(4,2) = -x_kk1[5] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(4,3) = -x_kk1[4] * x_kk1[3] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;
	Q(4,4) = ahrs_ekf->config.sigma_r_sqr * dt + w_sqr * ahrs_ekf->config.sigma_r_sqr * dt3_3 + (x_kk1[3]*x_kk1[3]+x_kk1[5]*x_kk1[5]+x_kk1[6]*x_kk1[6])*ahrs_ekf->config.sigma_w_sqr*dt3_3;
	Q(4,5) = -x_kk1[4] * x_kk1[5] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;
	Q(4,6) = -x_kk1[4] * x_kk1[6] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;

	Q(5,0) = -x_kk1[6] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(5,1) = -x_kk1[3] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(5,2) = x_kk1[4] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(5,3) = -x_kk1[5] * x_kk1[3] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;
	Q(5,4) = -x_kk1[5] * x_kk1[4] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;
	Q(5,5) = ahrs_ekf->config.sigma_r_sqr * dt + w_sqr * ahrs_ekf->config.sigma_r_sqr * dt3_3 + (x_kk1[3]*x_kk1[3]+x_kk1[4]*x_kk1[4]+x_kk1[6]*x_kk1[6])*ahrs_ekf->config.sigma_w_sqr*dt3_3;
	Q(5,6) = -x_kk1[5] * x_kk1[6] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;

	Q(6,0) = x_kk1[5] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(6,1) = -x_kk1[4] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(6,2) = -x_kk1[3] * dt2_2 * ahrs_ekf->config.sigma_w_sqr;
	Q(6,3) = -x_kk1[6] * x_kk1[3] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;
	Q(6,4) = -x_kk1[6] * x_kk1[4] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;
	Q(6,5) = -x_kk1[6] * x_kk1[5] * dt3_3 * ahrs_ekf->config.sigma_w_sqr;
	Q(6,6) = ahrs_ekf->config.sigma_r_sqr * dt + w_sqr * ahrs_ekf->config.sigma_r_sqr * dt3_3 + (x_kk1[3]*x_kk1[3]+x_kk1[4]*x_kk1[4]+x_kk1[5]*x_kk1[5])*ahrs_ekf->config.sigma_w_sqr*dt3_3;

	// P(k,k-1) = F(k)*P(k-1,k-1)*F(k)' + Q(k)
	P = (F ^ P ^ F.transpose()) + Q;

	/*Mat<4,1> quat;
	for (i = 0; i < 4; ++i)
	{
		quat(i,0) = x_kk1(i+3,0);
	}
	Mat<4,1> quat_normalized;
	op::normalize(quat,quat_normalized);
	x_state = x_kk1;
	for (i = 3; i < 7; ++i)
	{
		x_state(i,0) = quat_normalized(i-3,0);
	}*/

	quat_t quat;
	quat.s = x_kk1(3,0);
	quat.v[0] = x_kk1(4,0);
	quat.v[1] = x_kk1(5,0);
	quat.v[2] = x_kk1(6,0);

	quat = quaternions_normalise(quat);
	x_state = x_kk1;
	x_state(3,0) = quat.s;
	x_state(4,0) = quat.v[0];
	x_state(5,0) = quat.v[1];
	x_state(6,0) = quat.v[2];
}

void ahrs_ekf_update_step_acc(ahrs_ekf_t* ahrs_ekf)
{
	uint16_t i;

	Mat<7,1> x_kk1 = x_state;

	Mat<3,1> z_acc;
	for (i = 0; i < 3; ++i)
	{
		z_acc(i,0) = ahrs_ekf->imu->scaled_accelero.data[i];
	}

	float acc_z_global = -1.0f;

	// h_acc(x(k,k-1))
	Mat<3,1> h_acc_xkk1;
	h_acc_xkk1(0,0) = -2.0f*(x_kk1(4,0)*x_kk1(6,0) - x_kk1(3,0)*x_kk1(5,0)) * acc_z_global;
	h_acc_xkk1(1,0) = -2.0f*(x_kk1(5,0)*x_kk1(6,0) + x_kk1(3,0)*x_kk1(4,0)) * acc_z_global;
	h_acc_xkk1(2,0) = -(1.0f - 2.0f*(x_kk1(4,0)*x_kk1(4,0) + x_kk1(5,0)*x_kk1(5,0))) * acc_z_global;

	// H_acc(k) = jacobian(h_acc(x(k,k-1)))
	Mat<3,7> H_acc_k;

	H_acc_k(0,3) = -2.0f * x_kk1(5,0) * acc_z_global;
	H_acc_k(0,4) = 2.0f * x_kk1(6,0) * acc_z_global;
	H_acc_k(0,5) = -2.0f * x_kk1(3,0) * acc_z_global;
	H_acc_k(0,6) = 2.0f * x_kk1(4,0) * acc_z_global;

	H_acc_k(1,3) = 2.0f * x_kk1(4,0) * acc_z_global;
	H_acc_k(1,4) = 2.0f * x_kk1(3,0) * acc_z_global;
	H_acc_k(1,5) = 2.0f * x_kk1(6,0) * acc_z_global;
	H_acc_k(1,6) = 2.0f * x_kk1(5,0) * acc_z_global;

	H_acc_k(2,3) = 0.0f;
	H_acc_k(2,4) = -4.0f * x_kk1(4,0) * acc_z_global;
	H_acc_k(2,5) = -4.0f * x_kk1(5,0) * acc_z_global;
	H_acc_k(2,6) = 0.0f;

	// Innovation y(k) = z(k) - h(x(k,k-1))
	Mat<3,1> yk_acc = z_acc - h_acc_xkk1;

	float acc_norm = vectors_norm(ahrs_ekf->imu->scaled_accelero.data);
	float acc_norm_diff = maths_f_abs(1.0f-acc_norm);
	float noise = 1.0f - maths_center_window_4(5.0f*acc_norm_diff);

	// Innovation covariance S(k) = H(k) * P(k,k-1) * H(k)' + R
	Mat<3,3> Sk_acc = (H_acc_k ^ P ^ H_acc_k.transpose()) + R_acc + Mat<3,3>(noise*0.1f,true);

	// Kalman gain: K(k) = P(k,k-1) * H(k)' * S(k)^-1
	Mat<3,3> Sk_inv;
	op::inverse(Sk_acc, Sk_inv);
	Mat<7,3> K_acc = P ^ (H_acc_k.transpose() ^ Sk_inv);

	// Updated state estimate: x(k,k) = x(k,k-1) + K(k)*y_k
	Mat<7,1> x_kk = x_kk1 + (K_acc ^ yk_acc);

	/*Mat<4,1> quat;
	for (i = 0; i < 4; ++i)
	{
		quat(i,0) = x_kk(i+3,0);
	}
	Mat<4,1> quat_normalized;
	op::normalize(quat,quat_normalized);
	x_state = x_kk;
	for (i = 3; i < 7; ++i)
	{
		x_state(i,0) = quat_normalized(i-3,0);
	}*/

	quat_t quat;
	quat.s = x_kk(3,0);
	quat.v[0] = x_kk(4,0);
	quat.v[1] = x_kk(5,0);
	quat.v[2] = x_kk(6,0);

	quat = quaternions_normalise(quat);
	x_state = x_kk;
	x_state(3,0) = quat.s;
	x_state(4,0) = quat.v[0];
	x_state(5,0) = quat.v[1];
	x_state(6,0) = quat.v[2];

	// Update covariance estimate
	P = (Id - (K_acc ^ H_acc_k)) ^ P;

}

void ahrs_ekf_update_step_mag(ahrs_ekf_t* ahrs_ekf)
{
	uint16_t i;

	Mat<7,1> x_kk1 = x_state;

	/*ahrs_ekf->imu->mag_global[0] = 1.0f;
	ahrs_ekf->imu->mag_global[1] = 0.0f;
	ahrs_ekf->imu->mag_global[2] = 0.0f;
	
	quat_t qtmp1 = quaternions_create_from_vector(ahrs_ekf->imu->scaled_compass.data); 
	quat_t qe_mag_global = quaternions_local_to_global(ahrs_ekf->ahrs->qe, qtmp1);
	//qe_mag_global.v[X] /= mag_norm;
	//qe_mag_global.v[Y] /= mag_norm;
	qe_mag_global.v[Z] = 0.0f;   // set z component in global frame to 0
	quat_t mag_corrected_local = quaternions_global_to_local(ahrs_ekf->ahrs->qe, qe_mag_global);		

	Mat<3,1> z_mag;
	for (i = 0; i < 3; ++i)
	{
		z_mag(i,0) = mag_corrected_local.v[i];
	}*/

	Mat<3,1> z_mag;
	for (i = 0; i < 3; ++i)
	{
		z_mag(i,0) = ahrs_ekf->imu->scaled_compass.data[i];
	}

	// h_mag(x(k,k-1))
	Mat<3,1> h_mag_xkk1;
	h_mag_xkk1(0,0) = (1.0f - 2.0f*(x_kk1(5,0)*x_kk1(5,0) + x_kk1(6,0)*x_kk1(6,0)))*ahrs_ekf->imu->mag_global[0] + 2.0f*(x_kk1(4,0)*x_kk1(6,0) - x_kk1(3,0)*x_kk1(5,0))*ahrs_ekf->imu->mag_global[2];
	h_mag_xkk1(1,0) = 2.0f*(x_kk1(4,0)*x_kk1(5,0) - x_kk1(3,0)*x_kk1(6,0))*ahrs_ekf->imu->mag_global[0] + 2.0f*(x_kk1(5,0)*x_kk1(6,0) + x_kk1(3,0)*x_kk1(4,0))*ahrs_ekf->imu->mag_global[2];
	h_mag_xkk1(2,0) = 2.0f*(x_kk1(4,0)*x_kk1(6,0) - x_kk1(3,0)*x_kk1(5,0))*ahrs_ekf->imu->mag_global[0] + (1.0f - 2.0f*(x_kk1(4,0)*x_kk1(4,0) + x_kk1(5,0)*x_kk1(5,0)))*ahrs_ekf->imu->mag_global[2];

	// H_mag(k) = jacobian(h_mag(x(k,k-1)))
	Mat<3,7> H_mag_k;

	H_mag_k(0,3) = -2.0f * x_kk1(5,0) * ahrs_ekf->imu->mag_global[2];
	H_mag_k(0,4) = 2.0f * x_kk1(6,0) * ahrs_ekf->imu->mag_global[2];
	H_mag_k(0,5) = -4.0f * x_kk1(5,0) * ahrs_ekf->imu->mag_global[0] - 2.0f * x_kk1(3,0) * ahrs_ekf->imu->mag_global[2];
	H_mag_k(0,6) = -4.0f * x_kk1(6,0) * ahrs_ekf->imu->mag_global[0] + 2.0f * x_kk1(4,0) * ahrs_ekf->imu->mag_global[2];

	H_mag_k(1,3) = -2.0f * x_kk1(6,0) * ahrs_ekf->imu->mag_global[0] + 2.0f * x_kk1(4,0) * ahrs_ekf->imu->mag_global[2];
	H_mag_k(1,4) = 2.0f * x_kk1(5,0) * ahrs_ekf->imu->mag_global[0] + 2.0f * x_kk1(3,0) * ahrs_ekf->imu->mag_global[2];
	H_mag_k(1,5) = 2.0f * x_kk1(4,0) * ahrs_ekf->imu->mag_global[0] + 2.0f * x_kk1(6,0) * ahrs_ekf->imu->mag_global[2];
	H_mag_k(1,6) = -2.0f * x_kk1(3,0) * ahrs_ekf->imu->mag_global[0] + 2.0f * x_kk1(5,0) * ahrs_ekf->imu->mag_global[2];

	H_mag_k(2,3) = -2.0f * x_kk1(5,0) * ahrs_ekf->imu->mag_global[0];
	H_mag_k(2,4) = 2.0f * x_kk1(6,0) * ahrs_ekf->imu->mag_global[0] - 4.0f * x_kk1(4,0) * ahrs_ekf->imu->mag_global[2];
	H_mag_k(2,5) = -2.0f * x_kk1(3,0) * ahrs_ekf->imu->mag_global[0] - 4.0f * x_kk1(5,0) * ahrs_ekf->imu->mag_global[2];
	H_mag_k(2,6) = 2.0f * x_kk1(4,0) * ahrs_ekf->imu->mag_global[0];

	// Innovation y(k) = z(k) - h(x(k,k-1))
	Mat<3,1> yk_mag = z_mag - h_mag_xkk1;

	// Innovation covariance S(k) = H(k) * P(k,k-1) * H(k)' + R
	Mat<3,3> Sk_mag = (H_mag_k ^ P ^ H_mag_k.transpose()) + R_mag;

	// Kalman gain: K(k) = P(k,k-1) * H(k)' * S(k)^-1
	Mat<3,3> Sk_inv;
	op::inverse(Sk_mag, Sk_inv);
	Mat<7,3> K_mag = P ^ (H_mag_k.transpose() ^ Sk_inv);

	// Updated state estimate: x(k,k) = x(k,k-1) + K(k)*y_k
	Mat<7,1> x_kk = x_kk1 + (K_mag ^ yk_mag);
	//Mat<7,1> x_kk = x_kk1;

	/*Mat<4,1> quat;
	for (i = 0; i < 4; ++i)
	{
		quat(i,0) = x_kk(i+3,0);
	}
	Mat<4,1> quat_normalized;
	op::normalize(quat,quat_normalized);
	x_state = x_kk;
	for (i = 3; i < 7; ++i)
	{
		x_state(i,0) = quat_normalized(i-3,0);
	}*/

	quat_t quat;
	quat.s = x_kk(3,0);
	quat.v[0] = x_kk(4,0);
	quat.v[1] = x_kk(5,0);
	quat.v[2] = x_kk(6,0);

	quat = quaternions_normalise(quat);
	x_state = x_kk;
	x_state(3,0) = quat.s;
	x_state(4,0) = quat.v[0];
	x_state(5,0) = quat.v[1];
	x_state(6,0) = quat.v[2];

	// Update covariance estimate
	P = (Id - (K_mag ^ H_mag_k)) ^ P;

}

bool ahrs_ekf_init(ahrs_ekf_t* ahrs_ekf, const ahrs_ekf_config_t* config, imu_t* imu, ahrs_t* ahrs)
{
	bool init_success = true;

	ahrs_ekf->imu = imu;
	ahrs_ekf->ahrs = ahrs;

	ahrs_ekf->config = *config;

	ahrs_ekf_init_cpp(ahrs_ekf);

	ahrs_ekf->ahrs->internal_state = AHRS_INIT;

	return init_success;
}

void ahrs_ekf_update(ahrs_ekf_t* ahrs_ekf)
{
	ahrs_ekf->ahrs->dt = ahrs_ekf->imu->dt;

	// To enable changing of R with onboard parameters.
	R_acc(0,0) = ahrs_ekf->config.R_acc;
	R_acc(1,1) = ahrs_ekf->config.R_acc;
	R_acc(2,2) = ahrs_ekf->config.R_acc;

	R_mag(0,0) = ahrs_ekf->config.R_mag;
	R_mag(1,1) = ahrs_ekf->config.R_mag;
	R_mag(2,2) = ahrs_ekf->config.R_mag;

	if (ahrs_ekf->imu->imu_ready)
	{
		ahrs_ekf_predict_step(ahrs_ekf);

		ahrs_ekf_update_step_acc(ahrs_ekf);
		ahrs_ekf_update_step_mag(ahrs_ekf);
		ahrs_ekf->ahrs->internal_state = AHRS_READY;
	}
	else
	{
		R_acc = Mat<3,3>(0.1f,true);
		R_mag = Mat<3,3>(10.0f,true);
		P = Mat<7,7>(1.0f,true);
		ahrs_ekf_update_step_acc(ahrs_ekf);
		ahrs_ekf_update_step_mag(ahrs_ekf);
		Mat<7,1> state_previous = x_state;

		ahrs_ekf_init_cpp(ahrs_ekf);

		for (int i = 3; i < 7; ++i)
		{
			x_state(i,0) = state_previous(i,0);
		}
		//x_state = state_previous;
	}
	

	uint16_t i;
	for (i = 0; i < 7; ++i)
	{
		ahrs_ekf->x[i] = x_state(i,0);
	}

	ahrs_ekf->ahrs->qe.s = x_state(3,0);
	ahrs_ekf->ahrs->qe.v[0] = x_state(4,0);
	ahrs_ekf->ahrs->qe.v[1] = x_state(5,0);
	ahrs_ekf->ahrs->qe.v[2] = x_state(6,0);

	#warning "remove before flight"
	ahrs_ekf->ahrs->angular_speed[X] = x_state(0,0);//ahrs_ekf->imu->scaled_gyro.data[X];// - //
	ahrs_ekf->ahrs->angular_speed[Y] = x_state(1,0);//ahrs_ekf->imu->scaled_gyro.data[Y];// - //
	ahrs_ekf->ahrs->angular_speed[Z] = x_state(2,0);//ahrs_ekf->imu->scaled_gyro.data[Z];// - //

	quat_t up, up_bf;
	up.s = 0; up.v[X] = UPVECTOR_X; up.v[Y] = UPVECTOR_Y; up.v[Z] = UPVECTOR_Z;
	up_bf = quaternions_global_to_local(ahrs_ekf->ahrs->qe, up);

	ahrs_ekf->ahrs->up_vec.v[X] = up_bf.v[X];
	ahrs_ekf->ahrs->up_vec.v[Y] = up_bf.v[Y];
	ahrs_ekf->ahrs->up_vec.v[Z] = up_bf.v[Z];

	ahrs_ekf->ahrs->linear_acc[X] = 9.81f * (ahrs_ekf->imu->scaled_accelero.data[X] - ahrs_ekf->ahrs->up_vec.v[X]) ;							// TODO: review this line!
	ahrs_ekf->ahrs->linear_acc[Y] = 9.81f * (ahrs_ekf->imu->scaled_accelero.data[Y] - ahrs_ekf->ahrs->up_vec.v[Y]) ;							// TODO: review this line!
	ahrs_ekf->ahrs->linear_acc[Z] = 9.81f * (ahrs_ekf->imu->scaled_accelero.data[Z] - ahrs_ekf->ahrs->up_vec.v[Z]) ;							// TODO: review this line!

	ahrs_ekf->ahrs->aero_attitude = coord_conventions_quat_to_aero(ahrs_ekf->ahrs->qe);

	uint16_t j;
	for (i = 0; i < 7; ++i)
	{
		for (j = 0; j < 7; ++j)
		{
			ahrs_ekf->ahrs->P_vect[i*7 + j] = P(i,j);
		}
	}
}