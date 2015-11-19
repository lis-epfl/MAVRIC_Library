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
 * \file servos_mix_adaptive_morph_telemetry.h
 * 
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *   
 * \brief Sends messages for the servo_mix_adaptive_morph module.
 *
 ******************************************************************************/


#ifndef SERVOS_MIX_WING_TELEMETRY_H_
#define SERVOS_MIX_WING_TELEMETRY_H_

#ifdef __cplusplus
	extern "C" {
#endif


#include "mavlink_stream.h"
#include "mavlink_message_handler.h"
#include "servos_mix_adaptive_morph.h"



/**
 * \brief							Initialize the servo mix
 * 
 * \param	mix						The pointer to the servo_mix structure
 * \param	mavlink_handler			The pointer to the MAVLink message handler
 *
 * \return	True if the init succeed, false otherwise
 */
bool servo_mix_adaptive_morph_telemetry_init(servo_mix_adaptive_morph_t* mix, mavlink_message_handler_t *mavlink_handler);



#ifdef __cplusplus
	}
#endif

#endif