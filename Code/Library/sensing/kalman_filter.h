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
*	\file ins.h
*	\brief Header file for the INS module
*/

#ifndef __KF_H__
#define __KF_H__

//--------------------
// Public definitions
//--------------------

#ifndef USE_KF
	#define USE_KF			0
#endif
//! Maximum dimension of the state or measure vector that will be fed to the kalman filter module
#define KF_MAX_STATE_MEASURE 7
    
// variable used for statistic purposes when in debug mode
#define KF_STATS_BUFFER 50

//-------------------------
// Public type definitions
//-------------------------

//! Type definition for the Kalman filter state
typedef struct
{
    int     nstates;
    int     nmeasures;
    short int debug;
    
	float* x;
	float* P;
	float* P_std;
	float* x_pr;
	float* P_pr;
	float* P_pr_std;
	float* phi;
	float* Q;
	float* H;
	float* R;
	float* K;
	float* z;
	float* h;
	
    int stats_buf_pos;
    float* stats_pr;
    float* stats_up;
    float stats_pr_tot;
    float stats_up_tot;
    float stats_up_prct;
}
kf_State;

/*!
*	Macro for Kalman filter structure initialization.
*   A structure kf_State has to be created beforehand, and given as argument of the macro.
*   The macro has to be called in a function 
*   (such as in a setup function, but not outside a function such as at the beginning of the file)
*/
#define KF_INIT(NS, NM, var)            \
    var.nstates     = NS;               \
    var.nmeasures   = NM;               \
    var.debug       = 0;                \
    var.stats_buf_pos   = 0;            \
    {                                   \
        static float x[NS];             \
        var.x = x;                      \
        static float P[NS*NS];          \
        var.P = P;                      \
    }                                    

/*!
*	Macro for Kalman filter structure with debug data initialization
*   More matrixes are initialized (takes more program memory!), but this allows to 
*   track the values of all matrixes and states.
*/
#define KF_INIT_DEBUG(NS, NM, var)      \
    var.nstates     = NS;               \
    var.nmeasures   = NM;               \
    var.debug       = 1;                \
    var.stats_buf_pos   = 0;            \
    {                                   \
        static float x[NS];             \
        var.x = x;                      \
        static float P[NS*NS];          \
        var.P = P;                      \
        static float P_std[NS];         \
        var.P_std = P_std;              \
        static float x_pr[NS];          \
        var.x_pr = x_pr;                \
        static float P_pr[NS*NS];       \
        var.P_pr = P_pr;                \
        static float P_pr_std[NS];      \
        var.P_pr_std = P_pr_std;        \
        static float phi[NS*NS];        \
        var.phi = phi;                  \
        static float Q[NS*NS];          \
        var.Q = Q;                      \
        static float H[NM*NS];          \
        var.H = H;                      \
        static float R[NM*NM];          \
        var.R = R;                      \
        static float K[NS*NM];          \
        var.K = K;                      \
        static float z[NS];             \
        var.z = z;                      \
        static float h[NS];             \
        var.h = h;                      \
                                                  \
        static float stats_pr[KF_STATS_BUFFER];   \
        stats_pr[0] = 0.0;                        \
        var.stats_pr = stats_pr;                  \
        static float stats_up[KF_STATS_BUFFER];   \
        var.stats_up = stats_up;                  \
    }

/** Errors module can throw */
enum kf_errors
{
	KF_ERROR_BASE = 0x3C00,
};

//----------------------------
// Public function prototypes
//----------------------------

void    kf_Predict(kf_State*, float*, float*, float*);
void    kf_Update (kf_State*, float*, float*, float*, float*, int);
float*  kf_GetX   (kf_State*);
float*  kf_GetP   (kf_State*);

#endif
