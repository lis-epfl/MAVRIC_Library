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
 * \file launch_detection.c
 * 
 * \author MAV'RIC Team
 * \author Dylan Bourgeois
 *   
 * \brief Detect launches
 *
 ******************************************************************************/

#include <stdlib.h>

#include "launch_detection.h"
#include "vectors.h"


#define ACC_X acc[0]		//< Alias for X acceleration
#define ACC_Y acc[1]		//< Alias for Y acceleration
#define ACC_Z acc[2]		//< Alias for Z acceleration

#define MIN_SAMPLES SAMPLING_PERIOD


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief 		Checks against threshold, if the minimum number of samples
 *				has been reached.
 *
 * \param ld 	Pointer to the launch detection struct
 *
 * \return 		1 if launch is detected
 * 				0 otherwise
 */
bool launch_detection_launch_detected(launch_detection_t * ld);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool launch_detection_launch_detected(launch_detection_t * ld)
{
	return ld->sma.nb_samples > MIN_SAMPLES ? ld->sma.current_avg < ld->c_idle + ld->t_launch : 0;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool launch_detection_init(launch_detection_t * ld, const launch_detection_conf_t* config)
{
 	sma_init(&ld->sma, SAMPLING_PERIOD);

 	ld->t_launch = config->t_launch;
 	ld->c_idle = config->c_idle;
 	ld->status = LAUNCH_IDLE;
 	ld->ACC_X = ld->ACC_Y = ld->ACC_Z = 0;
 	ld->enabled = 1;

 	return 1;
}

task_return_t launch_detection_update(launch_detection_t * ld, float acc[3])
{
 	ld->ACC_X = 1000.0f * acc[0];
 	ld->ACC_Y = 1000.0f * acc[1];
 	ld->ACC_Z = 1000.0f * acc[2];

 	int16_t acc_norm = (int16_t)(vectors_norm(ld->acc));
 	sma_update(&ld->sma, acc_norm);

 	if (launch_detection_launch_detected(ld))
 	{
 		ld->status = LAUNCHING;
 	}

 	return TASK_RUN_SUCCESS;
}
