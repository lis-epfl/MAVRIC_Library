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
#include "print_util.h"
#include "constants.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool ahrs_init(ahrs_t* ahrs)
{
	bool init_success = true;

	// Init structure
	ahrs->qe.s = 1.0f;
	ahrs->qe.v[X] = 0.0f;
	ahrs->qe.v[Y] = 0.0f;
	ahrs->qe.v[Z] = 0.0f;
	
	ahrs->angular_speed[X] = 0.0f;
	ahrs->angular_speed[Y] = 0.0f;
	ahrs->angular_speed[Z] = 0.0f;
	
	ahrs->linear_acc[X] = 0.0f;
	ahrs->linear_acc[Y] = 0.0f;
	ahrs->linear_acc[Z] = 0.0f;
	
	ahrs->internal_state = AHRS_UNLEVELED;

	ahrs->north_vec.s    = 0.0f;
	ahrs->north_vec.v[X] = 1.0f;
	ahrs->north_vec.v[Y] = 0.0f;
	ahrs->north_vec.v[Z] = 0.0f;
	
	ahrs->up_vec.s    = 0.0f;
	ahrs->up_vec.v[X] = 0.0f;
	ahrs->up_vec.v[Y] = 0.0f;
	ahrs->up_vec.v[Z] = -1.0f;
	
	print_util_dbg_print("[AHRS] Initialised.\r\n");
	
	return init_success;
}