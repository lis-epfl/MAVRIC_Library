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
 * \file imu.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Gregoire Heitz
 *   
 * \brief This file implements the IMU data structure
 *
 ******************************************************************************/


#include "imu.h"

#include "time_keeper.h"
#include "print_util.h"
#include "constants.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Computes the oriented sensors values from the raw sensor values
 *
 * \param	imu		Pointer structure of the imu
 */
static void imu_raw2oriented(imu_t *imu);


/**
 * \brief	Computes the scaled sensors values from the oriented sensor values
 * 
 * \param	imu		Pointer structure of the imu
 */
static void imu_oriented2scale(imu_t *imu);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void imu_raw2oriented(imu_t *imu)
{	
	uint16_t i;
	
	for (i=0; i<3; i++)
	{
		imu->oriented_gyro.data[i]		= imu->raw_gyro.data[imu->calib_gyro.axis[i]]     * imu->calib_gyro.orientation[i];
		imu->oriented_accelero.data[i]  = imu->raw_accelero.data[imu->calib_accelero.axis[i]] * imu->calib_accelero.orientation[i];
		imu->oriented_compass.data[i]	= imu->raw_compass.data[imu->calib_compass.axis[i]]  * imu->calib_compass.orientation[i];
	}
	
	/*if (imu->calib_gyro.calibration)
	{
		for (i=0; i<3; i++)
		{
			imu->calib_gyro.max_oriented_values[i] = maths_f_max(imu->calib_gyro.max_oriented_values[i],imu->oriented_gyro.data[i]);
			imu->calib_gyro.min_oriented_values[i] = maths_f_min(imu->calib_gyro.min_oriented_values[i],imu->oriented_gyro.data[i]);
}
	}*/

	/*if (imu->calib_accelero.calibration)
	{
		for (i=0; i<3; i++)
		{
			imu->calib_accelero.max_oriented_values[i] = maths_f_max(imu->calib_accelero.max_oriented_values[i],imu->oriented_accelero.data[i]);
			imu->calib_accelero.min_oriented_values[i] = maths_f_min(imu->calib_accelero.min_oriented_values[i],imu->oriented_accelero.data[i]);
		}
	}*/

	if (imu->calib_compass.calibration)
	{
		for (i=0; i<3; i++)
		{
			imu->calib_compass.max_oriented_values[i] = maths_f_max(imu->calib_compass.max_oriented_values[i],imu->oriented_compass.data[i]);
			imu->calib_compass.min_oriented_values[i] = maths_f_min(imu->calib_compass.min_oriented_values[i],imu->oriented_compass.data[i]);
		}
	}
}


static void imu_oriented2scale(imu_t *imu)
{
	for (int16_t i = 0; i < 3; i++)
	{
		imu->scaled_gyro.data[i]  		= (1.0f - GYRO_LPF) * imu->scaled_gyro.data[i] 		+ GYRO_LPF * ( ( imu->oriented_gyro.data[i]     - imu->calib_gyro.bias[i]     ) * imu->calib_gyro.scale_factor[i]     );
		imu->scaled_accelero.data[i]   	= (1.0f - ACC_LPF)  * imu->scaled_accelero.data[i] 	+ ACC_LPF  * ( ( imu->oriented_accelero.data[i] - imu->calib_accelero.bias[i] ) * imu->calib_accelero.scale_factor[i] );
		imu->scaled_compass.data[i] 	= (1.0f - MAG_LPF)  * imu->scaled_compass.data[i] 	+ MAG_LPF  * ( ( imu->oriented_compass.data[i]  - imu->calib_compass.bias[i]  ) * imu->calib_compass.scale_factor[i]  );
	}
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void imu_init (imu_t *imu, imu_conf_t *conf_imu, state_t* state)
{	
	//init dependency
	imu->state = state;
	
	//init gyro
	imu->calib_gyro.scale_factor[X] =  1.0f / conf_imu->gyroscope.scale_factor[X];
	imu->calib_gyro.scale_factor[Y] =  1.0f / conf_imu->gyroscope.scale_factor[Y];
	imu->calib_gyro.scale_factor[Z] =  1.0f / conf_imu->gyroscope.scale_factor[Z];
	imu->calib_gyro.bias[X]			= conf_imu->gyroscope.bias[X];
	imu->calib_gyro.bias[Y]			= conf_imu->gyroscope.bias[Y];
	imu->calib_gyro.bias[Z]			= conf_imu->gyroscope.bias[Z];
	imu->calib_gyro.orientation[X]	= conf_imu->gyroscope.orientation[X];
	imu->calib_gyro.orientation[Y]	= conf_imu->gyroscope.orientation[Y];
	imu->calib_gyro.orientation[Z]	= conf_imu->gyroscope.orientation[Z];
	imu->calib_gyro.axis[X]			= conf_imu->gyroscope.axis[X];
	imu->calib_gyro.axis[Y]			= conf_imu->gyroscope.axis[Y];
	imu->calib_gyro.axis[Z]			= conf_imu->gyroscope.axis[Z];
	imu->calib_gyro.max_oriented_values[X] = -10000.0f;
	imu->calib_gyro.max_oriented_values[Y] = -10000.0f;
	imu->calib_gyro.max_oriented_values[Z] = -10000.0f;
	imu->calib_gyro.min_oriented_values[X] =  10000.0f;
	imu->calib_gyro.min_oriented_values[Y] =  10000.0f;
	imu->calib_gyro.min_oriented_values[Z] =  10000.0f;
	imu->calib_gyro.calibration = false;
	
	//init accelero
	imu->calib_accelero.scale_factor[X]		=  1.0f / conf_imu->accelerometer.scale_factor[X];
	imu->calib_accelero.scale_factor[Y]		=  1.0f / conf_imu->accelerometer.scale_factor[Y];
	imu->calib_accelero.scale_factor[Z]		=  1.0f / conf_imu->accelerometer.scale_factor[Z];
	imu->calib_accelero.bias[X]				= conf_imu->accelerometer.bias[X];
	imu->calib_accelero.bias[Y]				= conf_imu->accelerometer.bias[Y];
	imu->calib_accelero.bias[Z]				= conf_imu->accelerometer.bias[Z];
	imu->calib_accelero.orientation[X]		= conf_imu->accelerometer.orientation[X];
	imu->calib_accelero.orientation[Y]		= conf_imu->accelerometer.orientation[Y];
	imu->calib_accelero.orientation[Z]		= conf_imu->accelerometer.orientation[Z];
	imu->calib_accelero.axis[X]				= conf_imu->accelerometer.axis[X];
	imu->calib_accelero.axis[Y]				= conf_imu->accelerometer.axis[Y];
	imu->calib_accelero.axis[Z]				= conf_imu->accelerometer.axis[Z];
	imu->calib_accelero.max_oriented_values[X] = -10000.0f;
	imu->calib_accelero.max_oriented_values[Y] = -10000.0f;
	imu->calib_accelero.max_oriented_values[Z] = -10000.0f;
	imu->calib_accelero.min_oriented_values[X] =  10000.0f;
	imu->calib_accelero.min_oriented_values[Y] =  10000.0f;
	imu->calib_accelero.min_oriented_values[Z] =  10000.0f;
	imu->calib_accelero.calibration = false;
	
	//init compass
	imu->calib_compass.scale_factor[X]		=  1.0f / conf_imu->magnetometer.scale_factor[X];
	imu->calib_compass.scale_factor[Y]		=  1.0f / conf_imu->magnetometer.scale_factor[Y];
	imu->calib_compass.scale_factor[Z]		=  1.0f / conf_imu->magnetometer.scale_factor[Z];
	imu->calib_compass.bias[X]				= conf_imu->magnetometer.bias[X];
	imu->calib_compass.bias[Y]				= conf_imu->magnetometer.bias[Y];
	imu->calib_compass.bias[Z]				= conf_imu->magnetometer.bias[Z];
	imu->calib_compass.orientation[X]		= conf_imu->magnetometer.orientation[X];
	imu->calib_compass.orientation[Y]		= conf_imu->magnetometer.orientation[Y];
	imu->calib_compass.orientation[Z]		= conf_imu->magnetometer.orientation[Z];
	imu->calib_compass.axis[X]				= conf_imu->magnetometer.axis[X];
	imu->calib_compass.axis[Y]				= conf_imu->magnetometer.axis[Y];
	imu->calib_compass.axis[Z]				= conf_imu->magnetometer.axis[Z];
	imu->calib_compass.max_oriented_values[X] = -10000.0f;
	imu->calib_compass.max_oriented_values[Y] = -10000.0f;
	imu->calib_compass.max_oriented_values[Z] = -10000.0f;
	imu->calib_compass.min_oriented_values[X] =  10000.0f;
	imu->calib_compass.min_oriented_values[Y] =  10000.0f;
	imu->calib_compass.min_oriented_values[Z] =  10000.0f;
	imu->calib_compass.calibration = false;
	
	imu->last_update = time_keeper_get_time_ticks();
	imu->dt = 0.004;
	
	print_util_dbg_print("[IMU] Initialisation\r\n");
}


void imu_update(imu_t *imu)
{
	uint32_t t = time_keeper_get_time_ticks();
	
	imu->dt = time_keeper_ticks_to_seconds(t - imu->last_update);
	imu->last_update = t;

	imu_raw2oriented(imu);
	imu_oriented2scale(imu);
}