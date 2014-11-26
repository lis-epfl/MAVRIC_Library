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
 * \file turnigy.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file configures the channels and the direction of the Turnigy remote
 * 
 ******************************************************************************/


#ifndef TURNIGY_H_
#define TURNIGY_H_

#ifdef __cplusplus
extern "C" {
#endif


#define RC_THROTTLE		0						///< The throttle channel
#define RC_THROTTLE_DIR 1						///< The direction of the throttle channel

#define RC_ROLL			1						///< The roll axis channel
#define RC_ROLL_DIR     1						///< The direction of the roll axis channel

#define RC_PITCH		2						///< The pitch axis channel
#define RC_PITCH_DIR    -1						///< The direction of the pitch axis channel

#define RC_YAW			3						///< The yaw axis channel
#define RC_YAW_DIR		1						///< The direction of the yaw axis channel

#define RC_SAFETY   4							///< The safety switch channel
#define RC_ID_MODE  5							///< The ID mode switch channel
#define RC_TRIM_P3  6							///< The P3 trim channel
#define RC_TRIM_P1  7							///< The P1 trim channel
#define RC_TRIM_P2  8							///< The P2 trim channel

#define DEADZONE 30.0f							///< The deadzone of the remote

#define RC_SCALEFACTOR 1.0f / 880.0f			///< The scale factor of remote inputs


#ifdef __cplusplus
}
#endif

#endif //TURNIGY_H_