/*
  Copyright (C) 2009. LIS Laboratory, EPFL, Lausanne

  This file is part of Aeropic.

  Aeropic is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 2.1 of the License, or
  (at your option) any later version.

  Aeropic is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with Aeropic.  If not, see <http://www.gnu.org/licenses/>.
*/
/*!
*	\file kalman_filter.c
*	\brief Source file for the floating point extended kalman filter
*
*	This module executes the extended kalman filter algorithm.
*
*   Make sure the following files are included to your project :
*   - 'program files\microchip\MPLAB C30\src\dsp\include\dsp.h'
*   - 'program files\microchip\MPLAB C30\src\dsp\src\minv.c'
*/

//----------
// Includes
//----------

// include common library files
#include "kalman_filter.h"
#include "matrixlib_float.h"
#include "linear_algebra.h"
//-------------------
// Private variables
//-------------------

float kf_t1[KF_MAX_STATE_MEASURE*KF_MAX_STATE_MEASURE];
float kf_t2[KF_MAX_STATE_MEASURE*KF_MAX_STATE_MEASURE];
float kf_t3[KF_MAX_STATE_MEASURE*KF_MAX_STATE_MEASURE];

//---------------------------------
// Public functions implementation
//---------------------------------

/*!
*	Prediction step of the Extended Kalman filter
*/
void kf_Predict(kf_State* kf, float* f, float* phi, float* Q)
{
    if(kf->debug == 1)
    {
        matf_copy(kf->nstates, 1,           f,   kf->x_pr);
        matf_copy(kf->nstates, kf->nstates, phi, kf->phi);
        matf_copy(kf->nstates, kf->nstates, Q,   kf->Q);
        
        kf->stats_pr[kf->stats_buf_pos] += matf_norm(kf->nstates, matf_sub(kf->nstates, 1, f, kf->x, kf_t1));
    }
    
    matf_copy(kf->nstates, 1, f, kf->x);
     
    // update of P
    matf_multiply_Bt (kf->nstates, kf->nstates, kf->nstates, kf->P,   phi,   kf_t1);      // temp1 = P*phi'
    matf_multiply    (kf->nstates, kf->nstates, kf->nstates, phi,     kf_t1, kf_t2);      // temp2 = phi*temp1 = phi*P*phi'
    matf_add         (kf->nstates, kf->nstates,              kf_t2,   Q,     kf->P);      // P  = temp2 + Q = phi*P*phi' + Q
    
    
    if(kf->debug == 1)
    {
	    matf_std (kf->nstates, kf->P, kf->P_std);        // if update step is called after each predict step, this gets erased
        matf_copy(kf->nstates, kf->nstates, kf->P,   kf->P_pr);
	    matf_std (kf->nstates, kf->P_pr, kf->P_pr_std);
    }    
}


/*!
*	Update step of the Extended Kalman filter. number of elements in z can be set by nmeasures if different from kf->nmeasures (otherwise, set 0)
*/
void kf_Update(kf_State* kf, float* H, float* R, float* z, float* h, int nmeasures)
{
    float H_times_x[KF_MAX_STATE_MEASURE];
    
    if (nmeasures == 0) nmeasures = kf->nmeasures;
    
    // computes K
    matf_multiply_Bt(kf->nstates,   kf->nstates,   nmeasures, kf->P, H,     kf_t3);   // temp3 = P*H'       (temp3 is used twice to compute K)
    matf_multiply   (nmeasures, kf->nstates,   nmeasures, H,     kf_t3, kf_t2);   // temp2 = H*temp3 = H*P*H'
    matf_add        (nmeasures, nmeasures,                kf_t2, R,     kf_t1);   // temp1 = temp2 + R = H*P*H' + R
    if(nmeasures == 1)
	{
    	kf_t2[0] = 1/kf_t1[0];
    }else{
    	//matf_invert     (nmeasures,                           kf_t2, kf_t1);   // temp2 = inv(temp1) = inv(H*P*H' + R)
		matrix_3x3_t temp3;
		matrix_6x6_t temp6;
		int i,j;
		
		switch(nmeasures)
		{
			case 3:
				for (i=0;i<3;i++)
				{
					for (j=0;j<3;j++)
					{
						temp3.v[i][j] = kf_t2[3*i+j];
					}
				}
				//temp3.v[0][0] = kf_t2[0];
				//temp3.v[0][1] = kf_t2[1];
				//temp3.v[0][2] = kf_t2[2];
				//temp3.v[1][0] = kf_t2[3];
				//temp3.v[1][1] = kf_t2[4];
				//temp3.v[1][2] = kf_t2[5];
				//temp3.v[2][0] = kf_t2[6];
				//temp3.v[2][1] = kf_t2[7];
				//temp3.v[2][2] = kf_t2[8];
				
				temp3 = inv3(temp3);
				
				for (i=0;i<3;i++)
				{
					for (j=0;j<3;j++)
					{
						kf_t2[3*i+j] = temp3.v[i][j];
					}
				}
				//kf_t1[0] = temp3.v[0][0];
				//kf_t1[1] = temp3.v[0][1];
				//kf_t1[2] = temp3.v[0][2];
				//kf_t1[3] = temp3.v[1][0];
				//kf_t1[4] = temp3.v[1][1];
				//kf_t1[5] = temp3.v[1][2];
				//kf_t1[6] = temp3.v[2][0];
				//kf_t1[7] = temp3.v[2][1];
				//kf_t1[8] = temp3.v[2][2];
				break;
			case 6:
				for (i=0;i<6;i++)
				{
					for (j=0;j<6;j++)
					{
						temp6.v[i][j] = kf_t2[6*i + j];
					}
				}
				//temp6 = inv6(temp6);
				for (i=0;i<6;i++)
				{
					for (j=0;j<6;j++)
					{
						kf_t2[6*i + j] = temp6.v[i][j];
					}
				}
				break;
		}
	}
	matf_multiply   (kf->nstates,   nmeasures, nmeasures, kf_t3, kf_t2, kf_t1);   // temp1(=K) = temp3*temp2 = P*H'*inv(H*P*H' + R)       (temp1(=K) is used twice to compute x and P updates)
	
	if(h == 0)
	{
		matf_multiply  (nmeasures,   kf->nstates, 1, H,  kf->x, H_times_x);    // H_times_x = H * x
		h = H_times_x;
	}	
	
	// updates x	
	matf_sub       (nmeasures, 1,                z,      h,     kf_t2);    // temp2 = z - h
	matf_multiply  (kf->nstates,   nmeasures, 1, kf_t1,  kf_t2, kf_t3);    // temp3(=x_corr) = temp1(=K)*temp2 = K*(z-h)
	matf_add       (kf->nstates,   1,                kf->x,  kf_t3, kf->x);    // x = x + temp3(=x_corr)
    
    
	if(kf->debug == 1)
	{
		matf_copy(nmeasures, kf->nstates,   H,     kf->H);
		matf_copy(nmeasures, nmeasures, R,     kf->R);
		matf_copy(kf->nstates,   nmeasures, kf_t1, kf->K);
		matf_copy(nmeasures, 1,             z,     kf->z);
		matf_copy(nmeasures, 1,             h,     kf->h);
        
		// adds correction due to update to statistics vector
		kf->stats_up[kf->stats_buf_pos] = matf_norm(kf->nstates, kf_t3);
        
		// computes total corrections due to prediction and update
		kf->stats_pr_tot    = matf_sum(KF_STATS_BUFFER, kf->stats_pr);
		kf->stats_up_tot    = matf_sum(KF_STATS_BUFFER, kf->stats_up);
        
		kf->stats_up_prct = kf->stats_up_tot/(kf->stats_up_tot + kf->stats_pr_tot); // percentage
        
		// increments stats counter
		kf->stats_buf_pos++;
		if(kf->stats_buf_pos >= KF_STATS_BUFFER) kf->stats_buf_pos = 0;
            
		// sets next stat value of correction due to prediction to 0 
		kf->stats_pr[kf->stats_buf_pos] = 0.;
	}
    
	// updates P
	matf_zeros     (kf->nstates, kf->nstates,   kf_t2);
	matf_diag      (kf->nstates, kf->nstates,   kf_t2, 1., 1, kf->nstates);          // temp2 = I
	matf_multiply  (kf->nstates, nmeasures, kf->nstates, kf_t1, H    , kf_t3);   // temp3 = temp1(=K)*H
	matf_sub       (kf->nstates, kf->nstates,                kf_t2, kf_t3, kf_t1);   // temp1 = temp2 - temp3 = I - K*H
	matf_multiply  (kf->nstates, kf->nstates,   kf->nstates, kf_t1, kf->P, kf_t2);   // temp2(=P) = temp1*P = (I - K*H)*P_pr
	matf_copy      (kf->nstates, kf->nstates,   kf_t2      , kf->P);                 // P = temp2
    
	if(kf->debug == 1)
	matf_std (kf->nstates, kf->P, kf->P_std);
}

/*!
*	Returns a handle to the state vector
*/
float* kf_GetX (kf_State* kf)
{
    return kf->x;
}    

/*!
*	Returns a handle to the covariance matrix
*/
float* kf_GetP (kf_State* kf)
{
    return kf->P;
}

