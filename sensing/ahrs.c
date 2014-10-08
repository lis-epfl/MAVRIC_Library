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
 * \file ahrs.c
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief This file implements data structure for attitude estimate
 *
 ******************************************************************************/
 

#include "ahrs.h"
#include "conf_platform.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void ahrs_init(ahrs_t* ahrs, ahrs_config_t* config)
{
	// Init dependencies

	int32_t x = config->x;
	int32_t y = config->y;
	int32_t z = config->z;


	// Init structure
	ahrs->qe.s = 1.0f;
	ahrs->qe.v[0] = 0.0f;
	ahrs->qe.v[1] = 0.0f;
	ahrs->qe.v[2] = 0.0f;
	
	ahrs->angular_speed[x] = 0.0f;
	ahrs->angular_speed[y] = 0.0f;
	ahrs->angular_speed[z] = 0.0f;
	
	ahrs->linear_acc[x] = 0.0f;
	ahrs->linear_acc[y] = 0.0f;
	ahrs->linear_acc[z] = 0.0f;
	
	ahrs->north_vec.s    = 0.0f;
	ahrs->north_vec.v[0] = 1.0f;
	ahrs->north_vec.v[1] = 0.0f;
	ahrs->north_vec.v[2] = 0.0f;
	
	ahrs->up_vec.s    = 0.0f;
	ahrs->up_vec.v[0] = 0.0f;
	ahrs->up_vec.v[1] = 0.0f;
	ahrs->up_vec.v[2] = -1.0f;
}