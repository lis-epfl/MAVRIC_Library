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
 * \file constants.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief Useful constants
 *
 ******************************************************************************/


#ifndef MATH_UTIL_H_
#define MATH_UTIL_H_

#ifdef __cplusplus
extern "C" 
{
#endif


#define GRAVITY 9.81f			///< The gravity constant


/**
 * \brief Enumerates the X, Y and Z orientations 
 * according to the autopilot placement on the MAV
 */
typedef enum
{
	X = 0,
	Y = 1,
	Z = 2,
} constants_orientation_t;


/**
 * \brief Enumerates the Roll, Pitch and Yaw orientations 
 * according to the autopilot placement on the MAV
 */
typedef enum
{
	ROLL 	= 0,
	PITCH 	= 1,
	YAW 	= 2,
} constants_roll_pitch_yaw_t;


/**
 * \brief Enumerates the up vector orientation 
 * according to the autopilot placement on the MAV
 */
typedef enum
{
	UPVECTOR_X = 0,
	UPVECTOR_Y = 0,
	UPVECTOR_Z = -1,
} constants_upvector_t;


/**
 * \brief Enumerates ON/OFF switches
 */
typedef enum
{
	OFF = 0,
	ON 	= 1,
} constants_on_off_t;

#ifdef __cplusplus
}
#endif

#endif /* MATH_UTIL_H_ */