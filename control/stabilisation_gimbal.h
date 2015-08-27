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
 * \file stabilisation_copter.h
 * 
 * \author MAV'RIC Team
 * \author Alexandre Cherpillod
 *   
 * \brief This file handles the stabilization of the gimbal
 *
 ******************************************************************************/


#ifndef STABILISATION_GIMBAL_H_
#define STABILISATION_GIMBAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stabilisation.h"
#include "servos.h"
#include "stabilisation_copter.h"

/**
 * \brief							Main Controller for controlling and stabilizing the gimbal
 *
 * \param	stabilisation_copter	The stabilisation structure
 */
void stabilisation_gimbal(stabilisation_copter_t* stabilisation_copter);


/**
 * \brief							Mix to servo for gimbal servos
 *
 * \param	control					Pointer to controlling inputs
 * \param	servos					The array of servos structure
 */
void gimbal_stabilisation_mix_to_servos_quad(control_command_t *control, servos_t* servos);

#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_GIMBAL_H_ */