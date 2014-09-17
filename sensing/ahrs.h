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
 * \file ahrs.h
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief This file implements data structure for attitude estimate
 *
 ******************************************************************************/


#ifndef AHRS_H_
#define AHRS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "quaternions.h"


/**
 * \brief Structure containing the Attitude and Heading Reference System
 */
typedef struct
{
	quat_t		qe;							///< quaternion defining the Attitude estimation of the platform
	
	float		angular_speed[3];			///< Gyro rates
	float		linear_acc[3];				///< Acceleration WITHOUT gravity
	
	float		heading;					///< The heading of the platform
	quat_t		up_vec;						///< The quaternion of the up vector
	quat_t		north_vec;					///< The quaternion of the north vector
	
	uint32_t	last_update;				///< The time of the last IMU update in ms
	float		dt;							///< The time interval between two IMU updates
} ahrs_t;

/**
 * \brief Configuration for the AHRS structure
 */
typedef struct  
{
	int32_t x;								///< The mapping for the X axis
	int32_t y;								///< The mapping for the Y axis
	int32_t z;								///< The mapping for the Z axis
}ahrs_config_t;

/**
 * \brief   Initialiases the ahrs structure
 * 
 * \param	ahrs 				Pointer to ahrs structure
 * \param	config				Pointer to the config structure
 */
void ahrs_init(ahrs_t* ahrs, ahrs_config_t* config);


#ifdef __cplusplus
}
#endif

#endif /* AHRS_H_ */
