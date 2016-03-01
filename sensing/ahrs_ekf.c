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

#include "ahrs_ekf.h"
#include "matrixlib_float.h"
#include "constants.h"


void ahrs_efk_init(ahrs_efk_t* ahrs_ekf, imu_t* imu)
{
	ahrs_ekf->imu = imu;

	matf_zeros(7,1,ahrs_ekf->x);

	matf_zeros(7,7,ahrs_ekf->F);
	

	matf_diag(7, 7, ahrs_ekf->P, 100.0f, 1, 7);

	// Q = cov(del_w * del_w^T)
	matf_zeros(7,7,ahrs_ekf->Q);

}


void ahrs_ekf_predict(ahrs_efk_t* ahrs_ekf)
{
	uint8_t i;

	float dt = ahrs_efk->imu->dt;

	float w_x = ahrs_ekf->imu->scaled_gyro.data[X];
	float w_y = ahrs_ekf->imu->scaled_gyro.data[Y];
	float w_z = ahrs_ekf->imu->scaled_gyro.data[Z];

	float x_k1k1[7];
	for (i = 0; i < 7; ++i)
	{
		x_k1k1[i] = ahrs_ekf->x[i];
	}

	float x_kk1[7];
	// x(k,k-1) = f(x(k-1,k-1),u(k));
	x_kk1[0] = x_k1k1[0];
	x_kk1[1] = x_k1k1[1];
	x_kk1[2] = x_k1k1[2];
	x_kk1[3] = x_k1k1[3] + 0.5* (-(w_x-x_k1k1[0])*x_k1k1[4] - (w_y-x_k1k1[1])*x_k1k1[5] - (w_z-x_k1k1[2])*x_k1k1[6]) * dt;
	x_kk1[4] = x_k1k1[4] + 0.5* ((w_x-x_k1k1[0])*x_k1k1[3] + (w_z-x_k1k1[2])*x_k1k1[5] - (w_y-x_k1k1[1])*x_k1k1[6]) * dt;
	x_kk1[5] = x_k1k1[5] + 0.5* ((w_y-x_k1k1[1])*x_k1k1[3] - (w_z-x_k1k1[2])*x_k1k1[4] + (w_x-x_k1k1[0])*x_k1k1[6]) * dt;
	x_kk1[6] = x_k1k1[6] + 0.5* ((w_z-x_k1k1[2])*x_k1k1[3] + (w_y-x_k1k1[1])*x_k1k1[4] - (w_x-x_k1k1[0])*x_k1k1[5]) * dt;

	// F(k,k-1) = I + jacobian(x(k-1),u(k))*dt;
	ahrs_ekf->F[0] = 1.0f;
	ahrs_ekf->F[8] = 1.0f;
	ahrs_ekf->F[16] = 1.0f;

	ahrs_ekf->F[21] = x_k1k1[4] * dt;
	ahrs_ekf->F[22] = x_k1k1[5] * dt;
	ahrs_ekf->F[23] = x_k1k1[6] * dt;
	ahrs_ekf->F[24] = 1.0f; // 1.0f + 0.0f;
	ahrs_ekf->F[25] = -(w_x-x_k1k1[0]) * dt;
	ahrs_ekf->F[26] = -(w_y-x_k1k1[1]) * dt;
	ahrs_ekf->F[27] = -(w_z-x_k1k1[2]) * dt;
	
	ahrs_ekf->F[28] = 1.0f - x_k1k1[3] * dt;
	ahrs_ekf->F[29] = x_k1k1[6] * dt;
	ahrs_ekf->F[30] = x_k1k1[5] * dt;
	ahrs_ekf->F[31] = w_x-x_k1k1[0] * dt;
	ahrs_ekf->F[32] = 1.0f; // 1.0f + 0.0f;
	ahrs_ekf->F[33] = (w_z-x_k1k1[2]) * dt;
	ahrs_ekf->F[34] = -(w_y-x_k1k1[1]) * dt;
	
	ahrs_ekf->F[35] = 1.0f - x_k1k1[6] * dt;
	ahrs_ekf->F[36] = -x_k1k1[3] * dt;
	ahrs_ekf->F[37] = x_k1k1[4] * dt;
	ahrs_ekf->F[38] = (w_y-x_k1k1[1]) * dt;
	ahrs_ekf->F[39] = -(w_z-x_k1k1[2]) * dt;
	ahrs_ekf->F[40] = 1.0f; // 1.0f + 0.0f;
	ahrs_ekf->F[41] = (w_x-x_k1k1[0]) * dt;
	
	ahrs_ekf->F[42] = x_k1k1[5] * dt;
	ahrs_ekf->F[43] = -x_k1k1[4] * dt;
	ahrs_ekf->F[44] = x_k1k1[3] * dt;
	ahrs_ekf->F[45] = (w_z-x_k1k1[2]) * dt;
	ahrs_ekf->F[46] = (w_y-x_k1k1[1]) * dt;
	ahrs_ekf->F[47] = -(w_x-x_k1k1[0]) * dt;
	ahrs_ekf->F[48] = 1.0f; // 1.0f + 0.0f;

	// P(k,k-1)

	float PkFkt[49];

	matf_multiply_Bt(7,7,7, ahrs_ekf->P, ahrs_ekf->F, PkFkt);

	float FkPkFkt[49];
	matf_multiply(7, 7, 7, ahrs_ekf->F, PkFkt, FkPkFkt);

	float Pk[49];

	matf_add(7,7, FkPkFkt, ahrs_ekf->Q, Pk);



	for (i = 0; i < 7; ++i)
	{
		ahrs_ekf->x[i] = x_k[i];
	}
}

void ahrs_ekf_update(ahrs_efk_t* ahrs_ekf)
{
	uint16_t i, j;

	float mag_global[3];

	mag_global[0] = cos(63.0f/180.0f*PI);
	mag_global[1] = 0.0f;
	mag_global[2] = sin(63.0f/180.0f*PI);

	float z_acc[3];
	for (i = 0; i < 3; ++i)
	{
		z_acc[i] = ahrs_ekf->imu->scaled_accelero[i];
	}
	
	float z_mag[3];
	for (i = 0; i < 3; ++i)
	{
		z_mag[i] = ahrs_ekf->imu->scaled_compass[i];
	}

	// h_acc(x(k,k-1))
	float h_acc_xkk1[3];
	h_acc_xkk1[0] = 2.0f*(x_kk1[4]*x_kk1[6] - x_kk1[3]*x_kk1[5]);
	h_acc_xkk1[1] = 2.0f*(x_kk1[5]*x_kk1[6] + x_kk1[3]*x_kk1[4]);
	h_acc_xkk1[2] = (1.0f - 2.0f*(x_kk1[4]*x_kk1[4] + x_kk1[5]*x_kk1[5]));

	// h_mag(x(k,k-1))
	float h_mag_xkk1[3];
	h_mag_xkk1[0] = (1.0f - 2.0f*(x_kk1[5]*x_kk1[5] + x_kk1[6]*x_kk1[6])*mag_global[0] + 2.0f*(x_kk1[4]*x_kk1[6] - x_kk1[3]*x_kk1[5])*mag_global[2]);
	h_mag_xkk1[1] = 2.0f*(x_kk1[4]*x_kk1[5] - x_kk1[3]*x_kk1[6])*mag_global[0] + 2.0f*(x_kk1[5]*x_kk1[6] + x_kk1[3]*x_kk1[4])*mag_global[2]);
	h_mag_xkk1[2] = 2.0f*(x_kk1[4]*x_kk1[6] - x_kk1[3]*x_kk1[5])*mag_global[0] + (1.0f - 2.0f*(x_kk1[4]*x_kk1[4] + x_kk1[5]*x_kk1[5]))*mag_global[2];

	// H_acc(k) = jacobian(h_acc(x(k,k-1)))
	float H_acc_k[21];

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			H_acc_k[i*7 + j] = 0.0f;
		}
	}

	H_acc_k[3] = 2.0f * x_kk1[5];
	H_acc_k[4] = -2.0 * x_kk1[6];
	H_acc_k[5] = 2.0f * x_kk1[3];
	H_acc_k[6] = -2.0f * x_kk1[4];

	H_acc_k[10] = -2.0f * x_kk1[4];
	H_acc_k[11] = -2.0f * x_kk1[3];
	H_acc_k[12] = -2.0f * x_kk1[6];
	H_acc_k[13] = -2.0f * x_kk1[5];

	H_acc_k[17] = 0.0f;
	H_acc_k[18] = 4.0f * x_kk1[4];
	H_acc_k[19] = 4.0f * x_kk1[5];
	H_acc_k[20] = 0.0f;

	// H_mag(k) = jacobian(h_mag(x(k,k-1)))
	float H_mag_k[21];

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			H_mag_k[i*7 + j] = 0.0f;
		}
	}

	H_mag_k[3] = -2.0f * x_kk1[5] * mag_global[2];
	H_mag_k[4] = 2.0f * x_kk1[6] * mag_global[2];
	H_mag_k[5] = -4.0f * x_kk1[5] * mag_global[0] - 2.0f * x_kk1[3] * mag_global[2];
	H_mag_k[6] = -4.0f * x_kk1[6] * mag_global[0] + 2.0f * x_kk1[4] * mag_global[2];

	H_mag_k[10] = -2.0f * x_kk1[6] * mag_global[0] + 2.0f * x_kk1[4] * mag_global[2];
	H_mag_k[11] = 2.0f * x_kk1[5] * mag_global[0] + 2.0f * x_kk1[3] * mag_global[2];
	H_mag_k[12] = 2.0f * x_kk1[4] * mag_global[0] + 2.0f * x_kk1[6] * mag_global[2];
	H_mag_k[13] = -2.0f * x_kk1[3] * mag_global[0] + 2.0f * x_kk1[5] * mag_global[2];

	H_mag_k[17] = -2.0f * x_kk1[5] * mag_global[0];
	H_mag_k[18] = 2.0f * x_kk1[6] * mag_global[0] - 4.0f * x_kk1[4] * mag_global[2];
	H_mag_k[19] = -2.0f * x_kk1[3] * mag_global[0] - 4.0f * x_kk1[5] * mag_global[2];
	H_mag_k[20] = 2.0f * x_kk1[4] * mag_global[0];

	
	
}

