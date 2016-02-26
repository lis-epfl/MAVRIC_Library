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
	matf_zeros(7,7,ahrs_ekf->P);

	matf_diag(7, 7, ahrs_ekf->P, 100.0f, 1, 7);



}


void ahrs_ekf_predict(ahrs_efk_t* ahrs_ekf)
{
	uint8_t i;

	float w_x = ahrs_ekf->imu->scaled_gyro.data[X];
	float w_y = ahrs_ekf->imu->scaled_gyro.data[Y];
	float w_z = ahrs_ekf->imu->scaled_gyro.data[Z];

	float x[7];
	for (i = 0; i < 7; ++i)
	{
		x[i] = ahrs_ekf->x[i];
	}

	float x_k[7];
	// x(k,k-1) = f(x(k-1,k-1),u(k));
	x_k[0] = x[0];
	x_k[1] = x[1];
	x_k[2] = x[2];
	x_k[3] = 0.5* (-(w_x-x[0])*x[4] - (w_y-x[1])*x[5] - (w_z-x[2])*x[6]);
	x_k[4] = 0.5* ((w_x-x[0])*x[3] + (w_z-x[2])*x[5] - (w_y-x[1])*x[6]);
	x_k[5] = 0.5* ((w_y-x[1])*x[3] - (w_z-x[2])*x[4] + (w_x-x[0])*x[6]);
	x_k[6] = 0.5* ((w_z-x[2])*x[3] + (w_y-x[1])*x[4] - (w_x-x[0])*x[5]);

	// F = jacobian(x(k-1),u(k));
	ahrs_ekf->F[21] = x_k[4];
	ahrs_ekf->F[22] = x_k[5];
	ahrs_ekf->F[23] = x_k[6];
	ahrs_ekf->F[24] = 0.0f;
	ahrs_ekf->F[25] = -(w_x-x_k[0]);
	ahrs_ekf->F[26] = -(w_y-x_k[1]);
	ahrs_ekf->F[27] = -(w_z-x_k[2]);
	
	ahrs_ekf->F[28] = -x_k[3];
	ahrs_ekf->F[29] = x_k[6];
	ahrs_ekf->F[30] = x_k[5];
	ahrs_ekf->F[31] = w_x-x_k[0];
	ahrs_ekf->F[32] = 0.0f;
	ahrs_ekf->F[33] = w_z-x_k[2];
	ahrs_ekf->F[34] = -(w_y-x_k[1]);
	
	ahrs_ekf->F[35] = -x_k[6];
	ahrs_ekf->F[36] = -x_k[3];
	ahrs_ekf->F[37] = x_k[4];
	ahrs_ekf->F[38] = w_y-x_k[1];
	ahrs_ekf->F[39] = -(w_z-x_k[2]);
	ahrs_ekf->F[40] = 0.0f;
	ahrs_ekf->F[41] = w_x-x_k[0];
	
	ahrs_ekf->F[42] = x_k[5];
	ahrs_ekf->F[43] = -x_k[4];
	ahrs_ekf->F[44] = x_k[3];
	ahrs_ekf->F[45] = w_z-x_k[2];
	ahrs_ekf->F[46] = w_y-x_k[1];
	ahrs_ekf->F[47] = -(w_x-x_k[0]);
	ahrs_ekf->F[48] = 0.0f;

	// P(k,k-1

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