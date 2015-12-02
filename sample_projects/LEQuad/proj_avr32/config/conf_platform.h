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
 * \file conf_platform.h
 * 
 * \author MAV'RIC Team
 * 
 * \brief  This file configures the imu for the rev 4 of the maveric autopilot
 *   
 ******************************************************************************/


#ifndef CONF_PLATFORM_H_
#define CONF_PLATFORM_H_

#ifdef __cplusplus
	extern "C" {
#endif  

#define MAVLINK_SYS_ID 0

///< Definitions of Platform configuration
#define M_REAR_LEFT 0		///< Define the index for the control
#define M_FRONT_LEFT 1		///< Define the index for the control
#define M_FRONT_RIGHT 2		///< Define the index for the control
#define M_REAR_RIGHT 3		///< Define the index for the control

#define M_FR_DIR ( 1)		///< Define the front right motor turn direction
#define M_FL_DIR (-1)		///< Define the front left motor turn direction
#define M_RR_DIR (-1)		///< Define the motor turn direction
#define M_RL_DIR ( 1)		///< Define the motor turn direction

#define M_FRONT 0			///< Define the index for the movement control to go front
#define M_RIGHT 1			///< Define the index for the movement control to go right
#define M_REAR 2			///< Define the index for the movement control to go backward
#define M_LEFT 3			///< Define the index for the movement control to go left

#define M_FRONT_DIR ( 1)	///< Define the direction of control
#define M_RIGHT_DIR (-1)	///< Define the direction of control
#define M_REAR_DIR  ( 1)	///< Define the direction of control
#define M_LEFT_DIR  (-1)	///< Define the direction of control

#define MIN_THRUST -0.9f	///< Define the minimum thrust to apply
#define MAX_THRUST 1.0f		///< Define the maximum thrust to apply

///< define if servos 7 and 8 are used
#define USE_SERVOS_7_8 false


#ifdef __cplusplus
}
#endif

#endif /* CONF_PLATFORM_H_ */
